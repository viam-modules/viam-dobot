// Real-time feedback (port 30004) reader for Dobot CR-series controllers.
//
// The controller broadcasts a packed 1440-byte little-endian struct every 8 ms
// (125 Hz). We parse only the fields the arm.Arm interface actually needs:
//   - q_actual          : 6 joint angles in DEGREES (offset 432)
//   - tool_vector_actual: TCP pose [x,y,z,rx,ry,rz] in mm + DEGREES (offset 624)
//   - robot_mode        : controller mode enum (offset 24)
//   - enable_status     : 1 if servos enabled (offset 1026)
//   - running_status    : 1 if executing motion (offset 1028)
//   - error_status      : 1 if alarm latched (offset 1029)
//
// A magic value at offset 48 (test_value = 0x123456789ABCDEF) is checked on
// every packet so we drop torn frames cheaply.
//
// The reader runs in its own goroutine, drains the socket, and atomically
// publishes the most recent valid frame to subscribers via a sync.RWMutex.

package arm

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"net"
	"strconv"
	"sync"
	"time"
)

const (
	feedbackPacketSize = 1440
	feedbackMagic      = 0x123456789ABCDEF
)

// Field offsets in the 1440-byte feedback frame (little-endian).
const (
	offRobotMode        = 24   // uint64
	offTestValue        = 48   // uint64
	offQActual          = 432  // float64[6] degrees
	offToolVectorActual = 624  // float64[6] (mm,mm,mm,deg,deg,deg)
	offEnableStatus     = 1026 // byte
	offRunningStatus    = 1028 // byte
	offErrorStatus      = 1029 // byte
)

// feedbackFrame is a snapshot of the most recent valid packet.
type feedbackFrame struct {
	JointDegs  [6]float64
	TCP        [6]float64 // mm,mm,mm, deg,deg,deg (Dobot Euler)
	Mode       int
	Enabled    bool
	Running    bool
	HasError   bool
	ReceivedAt time.Time
}

// feedbackClient reads the 30004 stream and publishes the latest frame.
type feedbackClient struct {
	host string
	port int

	mu    sync.RWMutex
	frame feedbackFrame
	have  bool
	err   error // last connection error, cleared on successful read

	cancelFn context.CancelFunc
	doneCh   chan struct{}
}

func newFeedbackClient(host string, port int) *feedbackClient {
	return &feedbackClient{host: host, port: port}
}

// start launches the background reader. ctx controls overall lifetime.
func (f *feedbackClient) start(parent context.Context) {
	ctx, cancel := context.WithCancel(parent)
	f.cancelFn = cancel
	f.doneCh = make(chan struct{})
	go f.run(ctx)
}

func (f *feedbackClient) stop() {
	if f.cancelFn != nil {
		f.cancelFn()
	}
	if f.doneCh != nil {
		<-f.doneCh
	}
}

// latest returns a snapshot of the last valid frame and whether one has been seen.
func (f *feedbackClient) latest() (feedbackFrame, bool, error) {
	f.mu.RLock()
	defer f.mu.RUnlock()
	return f.frame, f.have, f.err
}

// waitForFrame blocks until at least one frame has been parsed or ctx expires.
func (f *feedbackClient) waitForFrame(ctx context.Context) (feedbackFrame, error) {
	deadline := time.Now().Add(2 * time.Second)
	if d, ok := ctx.Deadline(); ok && d.Before(deadline) {
		deadline = d
	}
	for {
		fr, ok, err := f.latest()
		if ok {
			return fr, nil
		}
		if err != nil {
			return feedbackFrame{}, err
		}
		if !time.Now().Before(deadline) {
			return feedbackFrame{}, errors.New("timed out waiting for feedback frame")
		}
		select {
		case <-ctx.Done():
			return feedbackFrame{}, ctx.Err()
		case <-time.After(20 * time.Millisecond):
		}
	}
}

func (f *feedbackClient) run(ctx context.Context) {
	defer close(f.doneCh)
	backoff := 200 * time.Millisecond
	maxBackoff := 5 * time.Second

	for ctx.Err() == nil {
		conn, err := dialFeedback(ctx, f.host, f.port)
		if err != nil {
			f.recordError(err)
			if !sleepCtx(ctx, backoff) {
				return
			}
			backoff = nextBackoff(backoff, maxBackoff)
			continue
		}
		backoff = 200 * time.Millisecond
		f.readLoop(ctx, conn)
		_ = conn.Close()
	}
}

func dialFeedback(ctx context.Context, host string, port int) (net.Conn, error) {
	d := net.Dialer{Timeout: dialTimeout}
	return d.DialContext(ctx, "tcp", net.JoinHostPort(host, strconv.Itoa(port)))
}

func (f *feedbackClient) readLoop(ctx context.Context, conn net.Conn) {
	buf := make([]byte, feedbackPacketSize)
	for ctx.Err() == nil {
		_ = conn.SetReadDeadline(time.Now().Add(2 * time.Second))
		if _, err := io.ReadFull(conn, buf); err != nil {
			f.recordError(fmt.Errorf("feedback read: %w", err))
			return
		}
		if frame, ok := parseFeedback(buf); ok {
			f.mu.Lock()
			f.frame = frame
			f.have = true
			f.err = nil
			f.mu.Unlock()
		}
		// torn frame — let the loop continue; bad frames are common while
		// the CR controller is rebooting or after re-enable.
	}
}

// recordError marks the connection as failed and invalidates the cached frame
// so callers don't keep treating the last good frame as live state. This is
// what lets IsMoving and waitForMotionComplete notice that the controller is
// no longer reporting status — without it, a 30004 disconnect mid-motion would
// look like the move was still running forever.
func (f *feedbackClient) recordError(err error) {
	f.mu.Lock()
	f.err = err
	f.have = false
	f.mu.Unlock()
}

// parseFeedback extracts the fields we care about from a 1440-byte frame.
// Returns (frame, true) if the magic header validates, else (zero, false).
func parseFeedback(buf []byte) (feedbackFrame, bool) {
	if len(buf) < feedbackPacketSize {
		return feedbackFrame{}, false
	}
	magic := binary.LittleEndian.Uint64(buf[offTestValue:])
	if magic != feedbackMagic {
		return feedbackFrame{}, false
	}

	var fr feedbackFrame
	mode := binary.LittleEndian.Uint64(buf[offRobotMode:])
	fr.Mode = int(mode)
	fr.Enabled = buf[offEnableStatus] != 0
	fr.Running = buf[offRunningStatus] != 0
	fr.HasError = buf[offErrorStatus] != 0
	fr.ReceivedAt = time.Now()

	for i := 0; i < 6; i++ {
		fr.JointDegs[i] = readFloat64(buf, offQActual+i*8)
		fr.TCP[i] = readFloat64(buf, offToolVectorActual+i*8)
	}
	return fr, true
}

func readFloat64(buf []byte, off int) float64 {
	bits := binary.LittleEndian.Uint64(buf[off : off+8])
	return math.Float64frombits(bits)
}

func sleepCtx(ctx context.Context, d time.Duration) bool {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return false
	case <-t.C:
		return true
	}
}

func nextBackoff(cur, max time.Duration) time.Duration {
	n := cur * 2
	if n > max {
		return max
	}
	return n
}
