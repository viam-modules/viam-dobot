// Dashboard (port 29999) command/ack client for Dobot CR-series controllers.
//
// The protocol is plain ASCII: a request is one command like `EnableRobot()`
// or `MovJ(10,20,30,0,0,0)`, a response is `ErrorID,{ResultList},CommandName;`.
// One command may be in flight at a time, so all writes are serialized via
// the dashClient mutex. Reads use a line-bounded scanner that splits on the
// `;` terminator.

package arm

import (
	"bufio"
	"context"
	"fmt"
	"net"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	defaultDashboardPort  = 29999
	defaultFeedbackPort   = 30004
	dashboardWriteTimeout = 5 * time.Second
	dashboardReadTimeout  = 10 * time.Second
	dialTimeout           = 5 * time.Second
)

// dashClient is a synchronous request/response client for the dashboard port.
// It is safe for concurrent callers; commands are serialized.
type dashClient struct {
	host string
	port int

	mu      sync.Mutex // serializes one in-flight command
	connMu  sync.RWMutex
	conn    net.Conn
	scanner *bufio.Scanner
}

func newDashClient(host string, port int) *dashClient {
	return &dashClient{host: host, port: port}
}

// connect opens (or reopens) the TCP connection to the dashboard.
func (c *dashClient) connect(ctx context.Context) error {
	c.connMu.Lock()
	defer c.connMu.Unlock()

	if c.conn != nil {
		_ = c.conn.Close()
		c.conn = nil
	}
	d := net.Dialer{Timeout: dialTimeout}
	conn, err := d.DialContext(ctx, "tcp", net.JoinHostPort(c.host, strconv.Itoa(c.port)))
	if err != nil {
		return fmt.Errorf("dashboard dial %s:%d: %w", c.host, c.port, err)
	}
	c.conn = conn
	c.scanner = bufio.NewScanner(conn)
	c.scanner.Buffer(make([]byte, 0, 4096), 1<<20)
	c.scanner.Split(splitOnSemicolon)
	return nil
}

func (c *dashClient) close() error {
	c.connMu.Lock()
	defer c.connMu.Unlock()
	if c.conn != nil {
		err := c.conn.Close()
		c.conn = nil
		return err
	}
	return nil
}

// send transmits cmd and returns the parsed response. cmd must NOT include the
// trailing newline. Format: `Cmd(arg1,arg2,...)`.
func (c *dashClient) send(ctx context.Context, cmd string) (*dashResp, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.connMu.RLock()
	conn := c.conn
	scanner := c.scanner
	c.connMu.RUnlock()
	if conn == nil {
		if err := c.connect(ctx); err != nil {
			return nil, err
		}
		c.connMu.RLock()
		conn = c.conn
		scanner = c.scanner
		c.connMu.RUnlock()
	}

	deadline, ok := ctx.Deadline()
	if !ok {
		deadline = time.Now().Add(dashboardReadTimeout)
	}
	_ = conn.SetWriteDeadline(time.Now().Add(dashboardWriteTimeout))
	_ = conn.SetReadDeadline(deadline)

	payload := cmd + "\n"
	if _, err := conn.Write([]byte(payload)); err != nil {
		_ = c.close()
		return nil, fmt.Errorf("dashboard write %q: %w", cmd, err)
	}
	if !scanner.Scan() {
		err := scanner.Err()
		if err == nil {
			err = fmt.Errorf("connection closed before response")
		}
		_ = c.close()
		return nil, fmt.Errorf("dashboard read response to %q: %w", cmd, err)
	}
	return parseDashResp(scanner.Text())
}

// dashResp is the parsed "ErrorID,{ResultList},Command;" reply.
type dashResp struct {
	ErrorID int
	Result  string // raw contents inside { }
	Command string // echoed command name
	Raw     string
}

// asInts decodes the comma-separated result list into ints.
func (r *dashResp) asInts() ([]int, error) {
	if r.Result == "" {
		return nil, nil
	}
	parts := strings.Split(r.Result, ",")
	out := make([]int, 0, len(parts))
	for _, p := range parts {
		v, err := strconv.Atoi(strings.TrimSpace(p))
		if err != nil {
			return nil, fmt.Errorf("non-int in result %q: %w", r.Result, err)
		}
		out = append(out, v)
	}
	return out, nil
}

// asFloats decodes the comma-separated result list into float64s.
func (r *dashResp) asFloats() ([]float64, error) {
	if r.Result == "" {
		return nil, nil
	}
	parts := strings.Split(r.Result, ",")
	out := make([]float64, 0, len(parts))
	for _, p := range parts {
		v, err := strconv.ParseFloat(strings.TrimSpace(p), 64)
		if err != nil {
			return nil, fmt.Errorf("non-float in result %q: %w", r.Result, err)
		}
		out = append(out, v)
	}
	return out, nil
}

// parseDashResp expects "ErrorID,{ResultList},CommandName" with the trailing
// semicolon already stripped by the scanner.
func parseDashResp(line string) (*dashResp, error) {
	line = strings.TrimSpace(line)
	if line == "" {
		return nil, fmt.Errorf("empty response")
	}
	open := strings.Index(line, "{")
	close := strings.Index(line, "}")
	if open < 0 || close < 0 || close < open {
		return nil, fmt.Errorf("malformed dashboard response: %q", line)
	}
	prefix := strings.TrimSuffix(strings.TrimSpace(line[:open]), ",")
	suffix := strings.TrimPrefix(strings.TrimSpace(line[close+1:]), ",")
	errID, err := strconv.Atoi(prefix)
	if err != nil {
		return nil, fmt.Errorf("malformed ErrorID in response %q: %w", line, err)
	}
	return &dashResp{
		ErrorID: errID,
		Result:  strings.TrimSpace(line[open+1 : close]),
		Command: suffix,
		Raw:     line,
	}, nil
}

// splitOnSemicolon is a bufio.SplitFunc that yields tokens delimited by `;`.
// Any trailing data without a `;` is returned at EOF as the final token.
func splitOnSemicolon(data []byte, atEOF bool) (advance int, token []byte, err error) {
	for i, b := range data {
		if b == ';' {
			return i + 1, data[:i], nil
		}
	}
	if atEOF && len(data) > 0 {
		return len(data), data, nil
	}
	return 0, nil, nil
}

// ---------------------------------------------------------------------------
// Dashboard command wrappers — return the parsed response and any error.
// All angle/Cartesian inputs are in DEGREES and MILLIMETERS to match the wire
// protocol; the arm.Arm wrapper converts from radians.
// ---------------------------------------------------------------------------

func (c *dashClient) enableRobot(ctx context.Context) error {
	return c.expectOK(ctx, "EnableRobot()")
}

func (c *dashClient) disableRobot(ctx context.Context) error {
	return c.expectOK(ctx, "DisableRobot()")
}

func (c *dashClient) clearError(ctx context.Context) error {
	return c.expectOK(ctx, "ClearError()")
}

// emergencyStop presses the E-stop (mode 1). This disables the arm AND latches
// an alarm; recovery requires releasing the E-stop (mode 0) and a ClearError().
// For a soft halt of in-flight motion, use stop() (Stop()) instead.
func (c *dashClient) emergencyStop(ctx context.Context) error {
	return c.expectOK(ctx, "EmergencyStop(1)")
}

// stop halts the in-flight motion command queue. The V4 wire command for this
// is "Stop()" (per Dobot-Arm/TCP-IP-Python-V4 dobot_api.py docstring: "Stop the
// delivered motion command queue or the RunScript command from running"). Do
// NOT use ResetRobot() here — it resets the entire robot state, not just halts
// motion.
func (c *dashClient) stop(ctx context.Context) error {
	return c.expectOK(ctx, "Stop()")
}

func (c *dashClient) speedFactor(ctx context.Context, percent int) error {
	if percent < 1 {
		percent = 1
	} else if percent > 100 {
		percent = 100
	}
	return c.expectOK(ctx, fmt.Sprintf("SpeedFactor(%d)", percent))
}

func (c *dashClient) velJ(ctx context.Context, percent int) error {
	return c.expectOK(ctx, fmt.Sprintf("VelJ(%d)", clampPct(percent)))
}

func (c *dashClient) accJ(ctx context.Context, percent int) error {
	return c.expectOK(ctx, fmt.Sprintf("AccJ(%d)", clampPct(percent)))
}

// jointMovJ moves PTP in joint space; angles are degrees.
func (c *dashClient) jointMovJ(ctx context.Context, j [6]float64) error {
	return c.expectOK(ctx, fmt.Sprintf(
		"MovJ(joint={%.4f,%.4f,%.4f,%.4f,%.4f,%.4f})",
		j[0], j[1], j[2], j[3], j[4], j[5]))
}

// robotMode returns the controller mode (1..11).
func (c *dashClient) robotMode(ctx context.Context) (int, error) {
	resp, err := c.send(ctx, "RobotMode()")
	if err != nil {
		return 0, err
	}
	if resp.ErrorID != 0 {
		return 0, fmt.Errorf("RobotMode returned ErrorID=%d", resp.ErrorID)
	}
	parts, err := resp.asInts()
	if err != nil || len(parts) == 0 {
		return 0, fmt.Errorf("RobotMode parse %q: %w", resp.Raw, err)
	}
	return parts[0], nil
}

// expectOK is a helper that errors on a non-zero ErrorID.
func (c *dashClient) expectOK(ctx context.Context, cmd string) error {
	resp, err := c.send(ctx, cmd)
	if err != nil {
		return err
	}
	if resp.ErrorID != 0 {
		return fmt.Errorf("%s ErrorID=%d (raw=%q)", strings.TrimSuffix(strings.SplitN(cmd, "(", 2)[0], " "), resp.ErrorID, resp.Raw)
	}
	return nil
}

func clampPct(p int) int {
	if p < 1 {
		return 1
	}
	if p > 100 {
		return 100
	}
	return p
}

// RobotMode constants (per Dobot CR-series TCP/IP protocol).
const (
	RobotModeInit       = 1
	RobotModeBrakeOpen  = 2
	RobotModePowerOff   = 3
	RobotModeDisabled   = 4
	RobotModeEnabled    = 5
	RobotModeBackdrive  = 6
	RobotModeRunning    = 7
	RobotModeSingleMove = 8
	RobotModeError      = 9
	RobotModePaused     = 10
	RobotModeCollision  = 11
)
