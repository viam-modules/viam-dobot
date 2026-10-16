package arm

import (
	"bufio"
	"context"
	"fmt"
	"net"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
)

// manualModeArm returns a cr10a wired to an in-process dashboard that acks every
// command, so SetManualMode/Stop can run without a controller. Any command whose
// wire form is listed in failCmds (e.g. "StartDrag()") is answered with a
// non-zero ErrorID instead, to drive the failure paths. These tests assert the
// manual-mode state the driver tracks (ManualMode, the auto-exit timer) — the
// wire encoding of StartDrag/StopDrag itself is covered by client_test.go.
func manualModeArm(t *testing.T, failCmds ...string) *cr10a {
	t.Helper()

	server, client := net.Pipe()
	t.Cleanup(func() {
		_ = server.Close()
		_ = client.Close()
	})

	fail := make(map[string]bool, len(failCmds))
	for _, c := range failCmds {
		fail[c] = true
	}

	c := &dashClient{conn: client}
	c.scanner = bufio.NewScanner(client)
	c.scanner.Split(splitOnSemicolon)

	go func() {
		r := bufio.NewReader(server)
		for {
			line, err := r.ReadString('\n')
			if err != nil {
				return
			}
			id := 0
			if fail[strings.TrimRight(line, "\r\n")] {
				id = 1
			}
			if _, err := server.Write([]byte(fmt.Sprintf("%d,{},ok;", id))); err != nil {
				return
			}
		}
	}()

	return &cr10a{
		logger: logging.NewTestLogger(t),
		opMgr:  operation.NewSingleOperationManager(),
		dash:   c,
	}
}

func manualOn(t *testing.T, a *cr10a) bool {
	t.Helper()
	on, err := a.ManualMode(context.Background(), nil)
	if err != nil {
		t.Fatalf("ManualMode: %v", err)
	}
	return on
}

func timerPending(a *cr10a) bool {
	a.manualExit.mu.Lock()
	defer a.manualExit.mu.Unlock()
	return a.manualExit.t != nil
}

// ---------- Properties ----------

func TestProperties(t *testing.T) {
	ctx := context.Background()

	live := &cr10a{}
	got, err := live.Properties(ctx, nil)
	if err != nil {
		t.Fatalf("cr10a.Properties: %v", err)
	}
	if want := (arm.Properties{SupportManualMode: true, SupportCartesianCommands: true}); got != want {
		t.Fatalf("cr10a.Properties = %+v, want %+v", got, want)
	}
	if manualOn(t, live) {
		t.Fatal("cr10a.ManualMode should be false before any SetManualMode")
	}

	sim := &simulatedCR10A{}
	got, err = sim.Properties(ctx, nil)
	if err != nil {
		t.Fatalf("simulatedCR10A.Properties: %v", err)
	}
	if want := (arm.Properties{SupportManualMode: false, SupportCartesianCommands: true}); got != want {
		t.Fatalf("simulatedCR10A.Properties = %+v, want %+v", got, want)
	}
	if _, err := sim.ManualMode(ctx, nil); err == nil {
		t.Fatal("simulatedCR10A.ManualMode: want unsupported error, got nil")
	}
	if err := sim.SetManualMode(ctx, true, 0, nil); err == nil {
		t.Fatal("simulatedCR10A.SetManualMode: want unsupported error, got nil")
	}
}

// ---------- exitTimer ----------

func TestExitTimerFires(t *testing.T) {
	var et exitTimer
	fired := make(chan struct{})
	et.schedule(10*time.Millisecond, func() { close(fired) })

	select {
	case <-fired:
	case <-time.After(time.Second):
		t.Fatal("timer did not fire within 1s")
	}
}

func TestExitTimerCancel(t *testing.T) {
	var et exitTimer
	fired := make(chan struct{})
	et.schedule(20*time.Millisecond, func() { close(fired) })
	et.cancel()

	select {
	case <-fired:
		t.Fatal("timer fired after cancel")
	case <-time.After(100 * time.Millisecond):
	}
	if et.t != nil {
		t.Fatal("exitTimer.t not nil after cancel")
	}
}

func TestExitTimerReschedule(t *testing.T) {
	var et exitTimer
	firstFired := make(chan struct{})
	secondFired := make(chan struct{})
	et.schedule(50*time.Millisecond, func() { close(firstFired) })
	et.schedule(10*time.Millisecond, func() { close(secondFired) })

	select {
	case <-secondFired:
	case <-time.After(time.Second):
		t.Fatal("rescheduled timer did not fire within 1s")
	}
	time.Sleep(100 * time.Millisecond)
	select {
	case <-firstFired:
		t.Fatal("first timer fired despite being rescheduled")
	default:
	}
}

// ---------- SetManualMode / ManualMode ----------

func TestSetManualModeEnterExit(t *testing.T) {
	a := manualModeArm(t)
	ctx := context.Background()

	if err := a.SetManualMode(ctx, true, 0, nil); err != nil {
		t.Fatalf("SetManualMode(true): %v", err)
	}
	if !manualOn(t, a) {
		t.Fatal("ManualMode false after enter")
	}
	if timerPending(a) {
		t.Fatal("enabledFor=0 must not schedule an auto-exit timer")
	}

	if err := a.SetManualMode(ctx, false, 0, nil); err != nil {
		t.Fatalf("SetManualMode(false): %v", err)
	}
	if manualOn(t, a) {
		t.Fatal("ManualMode true after exit")
	}
}

func TestSetManualModeEnabledForSchedulesTimerAndExitCancels(t *testing.T) {
	a := manualModeArm(t)
	ctx := context.Background()

	if err := a.SetManualMode(ctx, true, time.Hour, nil); err != nil {
		t.Fatalf("SetManualMode(true, 1h): %v", err)
	}
	if !timerPending(a) {
		t.Fatal("enabledFor>0 must schedule an auto-exit timer")
	}

	if err := a.SetManualMode(ctx, false, 0, nil); err != nil {
		t.Fatalf("SetManualMode(false): %v", err)
	}
	if timerPending(a) {
		t.Fatal("auto-exit timer still scheduled after SetManualMode(false)")
	}
}

// TestAutoExitManualMode drives the auto-exit callback directly, so the exit
// logic is verified without waiting on the wall clock (timer firing itself is
// covered by TestExitTimerFires).
func TestAutoExitManualMode(t *testing.T) {
	a := manualModeArm(t)
	ctx := context.Background()

	if err := a.SetManualMode(ctx, true, time.Hour, nil); err != nil {
		t.Fatalf("SetManualMode(true, 1h): %v", err)
	}

	a.autoExitManualMode()
	if manualOn(t, a) {
		t.Fatal("ManualMode true after auto-exit")
	}
	// A stale second firing must be a harmless no-op.
	a.autoExitManualMode()
	if manualOn(t, a) {
		t.Fatal("ManualMode true after redundant auto-exit")
	}
}

// TestSetManualModeTimedAutoExit exercises the real timer end to end: a short
// enabledFor must exit manual mode on its own.
func TestSetManualModeTimedAutoExit(t *testing.T) {
	a := manualModeArm(t)
	ctx := context.Background()

	if err := a.SetManualMode(ctx, true, 25*time.Millisecond, nil); err != nil {
		t.Fatalf("SetManualMode(true, 25ms): %v", err)
	}

	deadline := time.Now().Add(2 * time.Second)
	for manualOn(t, a) {
		if time.Now().After(deadline) {
			t.Fatal("manual mode still on after timed auto-exit")
		}
		time.Sleep(time.Millisecond)
	}
}

// ---------- Stop ----------

func TestStopExitsManualMode(t *testing.T) {
	a := manualModeArm(t)
	ctx := context.Background()

	if err := a.SetManualMode(ctx, true, time.Hour, nil); err != nil {
		t.Fatalf("SetManualMode(true): %v", err)
	}

	if err := a.Stop(ctx, nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if manualOn(t, a) {
		t.Fatal("ManualMode true after Stop")
	}
	if timerPending(a) {
		t.Fatal("auto-exit timer still scheduled after Stop")
	}
}

// ---------- DoCommand drag actions ----------

func TestDoCommandDragTracksState(t *testing.T) {
	a := manualModeArm(t)
	ctx := context.Background()

	if _, err := a.DoCommand(ctx, map[string]interface{}{"action": "start_drag"}); err != nil {
		t.Fatalf("DoCommand start_drag: %v", err)
	}
	if !manualOn(t, a) {
		t.Fatal("ManualMode not set after DoCommand start_drag")
	}

	// A pending auto-exit timer must be cancelled by stop_drag, so it cannot
	// later release an arm the user has re-engaged.
	a.manualExit.schedule(time.Hour, a.autoExitManualMode)
	if _, err := a.DoCommand(ctx, map[string]interface{}{"action": "stop_drag"}); err != nil {
		t.Fatalf("DoCommand stop_drag: %v", err)
	}
	if manualOn(t, a) {
		t.Fatal("ManualMode still set after DoCommand stop_drag")
	}
	if timerPending(a) {
		t.Fatal("auto-exit timer not cancelled by DoCommand stop_drag")
	}
}

// ---------- error paths: the tracked flag only changes on success ----------

func TestEnterManualModeDashNil(t *testing.T) {
	a := &cr10a{logger: logging.NewTestLogger(t)}
	if err := a.SetManualMode(context.Background(), true, 0, nil); err == nil {
		t.Fatal("SetManualMode(true) with nil dashboard: want error, got nil")
	}
	if a.manualMode.Load() {
		t.Fatal("manualMode set despite enter failure")
	}
}

func TestEnterManualModeStartDragError(t *testing.T) {
	a := manualModeArm(t, "StartDrag()")

	err := a.SetManualMode(context.Background(), true, 0, nil)
	if err == nil {
		t.Fatal("SetManualMode(true): want error from StartDrag, got nil")
	}
	if !strings.Contains(err.Error(), "clear_error") {
		t.Fatalf("error %q should hint at clear_error for a latched alarm", err)
	}
	if a.manualMode.Load() {
		t.Fatal("manualMode must stay false when StartDrag fails")
	}
}

func TestExitManualModeStopDragError(t *testing.T) {
	a := manualModeArm(t, "StopDrag()")
	ctx := context.Background()

	if err := a.SetManualMode(ctx, true, 0, nil); err != nil {
		t.Fatalf("SetManualMode(true): %v", err)
	}
	if err := a.SetManualMode(ctx, false, 0, nil); err == nil {
		t.Fatal("SetManualMode(false): want error from StopDrag, got nil")
	}
	// Exit failed, so the arm is (as far as we know) still in drag mode: the
	// tracked flag must not be optimistically cleared.
	if !a.manualMode.Load() {
		t.Fatal("manualMode cleared despite StopDrag failure")
	}
}
