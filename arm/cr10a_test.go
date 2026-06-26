package arm

import (
	"encoding/binary"
	"io"
	"math"
	"testing"

	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

// TestKinematicsParse loads the embedded JSON and exercises the resulting
// model with a zero pose to confirm the chain is well-formed.
func TestKinematicsParse(t *testing.T) {
	model, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, "cr10a")
	if err != nil {
		t.Fatalf("UnmarshalModelJSON: %v", err)
	}
	if got := len(model.DoF()); got != 6 {
		t.Fatalf("expected 6 DoF, got %d", got)
	}
	zero := make([]referenceframe.Input, 6)
	pose, err := model.Transform(zero)
	if err != nil {
		t.Fatalf("Transform(zero): %v", err)
	}
	pt := pose.Point()
	// At the home pose (joints all zero) the URDF places the TCP somewhere
	// out along the arm; the precise number depends on tool offset, but the
	// reach should be on the order of 1 m. Just assert finiteness here.
	if math.IsNaN(pt.X) || math.IsNaN(pt.Y) || math.IsNaN(pt.Z) {
		t.Fatalf("non-finite home pose %v", pt)
	}
	if math.Abs(pt.X)+math.Abs(pt.Y)+math.Abs(pt.Z) > 5000 {
		t.Fatalf("home pose unreasonably far from base: %v", pt)
	}
}

// TestFeedbackParse builds a synthetic 1440-byte packet with a known magic,
// joint vector, TCP vector, and status flags and confirms parseFeedback
// returns the right values.
func TestFeedbackParse(t *testing.T) {
	buf := make([]byte, feedbackPacketSize)

	// magic
	binary.LittleEndian.PutUint64(buf[offTestValue:], feedbackMagic)
	// robot mode = RobotModeRunning
	binary.LittleEndian.PutUint64(buf[offRobotMode:], uint64(RobotModeRunning))
	// status bytes
	buf[offEnableStatus] = 1
	buf[offRunningStatus] = 1
	buf[offErrorStatus] = 0

	// joint angles in degrees
	jointAngles := [6]float64{0, -90, 90, 0, 90, 0}
	for i, v := range jointAngles {
		binary.LittleEndian.PutUint64(buf[offQActual+i*8:], math.Float64bits(v))
	}
	// TCP pose
	tcp := [6]float64{500, 0, 600, 180, 0, 0}
	for i, v := range tcp {
		binary.LittleEndian.PutUint64(buf[offToolVectorActual+i*8:], math.Float64bits(v))
	}

	fr, ok := parseFeedback(buf)
	if !ok {
		t.Fatalf("parseFeedback rejected a valid packet")
	}
	if fr.Mode != RobotModeRunning {
		t.Fatalf("Mode: got %d want %d", fr.Mode, RobotModeRunning)
	}
	if !fr.Enabled || !fr.Running || fr.HasError {
		t.Fatalf("status: enabled=%v running=%v err=%v (want true,true,false)", fr.Enabled, fr.Running, fr.HasError)
	}
	if fr.JointDegs != jointAngles {
		t.Fatalf("joints: got %v want %v", fr.JointDegs, jointAngles)
	}
	if fr.TCP != tcp {
		t.Fatalf("tcp: got %v want %v", fr.TCP, tcp)
	}
}

// TestFeedbackRejectsBadMagic ensures torn frames are dropped silently.
func TestFeedbackRejectsBadMagic(t *testing.T) {
	buf := make([]byte, feedbackPacketSize)
	// leave magic zero
	if _, ok := parseFeedback(buf); ok {
		t.Fatalf("parseFeedback accepted a packet with bad magic")
	}
}

// TestFeedbackInvalidatesOnError covers the contract that a connection error
// invalidates any cached frame. Without this, the motion-completion poll and
// IsMoving would keep treating the last good frame as live state after the
// 30004 socket drops mid-motion, spinning forever on a stale Running=true.
func TestFeedbackInvalidatesOnError(t *testing.T) {
	f := newFeedbackClient("nowhere", 30004)

	// Simulate the read loop having published a live frame.
	f.mu.Lock()
	f.frame = feedbackFrame{Running: true}
	f.have = true
	f.mu.Unlock()

	if _, ok, _ := f.latest(); !ok {
		t.Fatalf("precondition: expected latest() to report a live frame")
	}

	// Simulate the read loop noticing a disconnect.
	f.recordError(io.EOF)

	if fr, ok, err := f.latest(); ok || err == nil {
		t.Fatalf("after recordError, latest() should report no live frame and surface the error; got frame=%+v ok=%v err=%v", fr, ok, err)
	}
}

// TestRadPerSecToPercent covers the conversion that maps planner-supplied
// joint velocity/accel limits (rad/s) to the controller's 1..100 SpeedJ/AccJ
// percent. Clamping at both ends matters because Dobot rejects values outside
// 1..100, and a planner request below 1% would otherwise round to 0.
func TestRadPerSecToPercent(t *testing.T) {
	cases := []struct {
		degPerSec float64
		want      int
	}{
		{180, 100}, // exactly 100%
		{90, 50},   // half
		{360, 100}, // clamped above
		{0.1, 1},   // clamped below — would round to 0 without the floor
		{0, 1},     // clamped below
	}
	for _, tc := range cases {
		got := radPerSecToPercent(rdkutils.DegToRad(tc.degPerSec), maxJointSpeedDegPerSec)
		if got != tc.want {
			t.Errorf("radPerSecToPercent(%g°/s) = %d%%, want %d%%", tc.degPerSec, got, tc.want)
		}
	}
}

// TestParseDashResp covers the typical response shapes from the Dobot
// dashboard port.
func TestParseDashResp(t *testing.T) {
	cases := []struct {
		in           string
		wantErr      int
		wantResult   string
		wantCommand  string
		wantParseErr bool
	}{
		{"0,{},EnableRobot()", 0, "", "EnableRobot()", false},
		{"0,{1.234,2.345,3.456,0,0,0},GetPose()", 0, "1.234,2.345,3.456,0,0,0", "GetPose()", false},
		{"5,{},MovJ(1,2,3,0,0,0)", 5, "", "MovJ(1,2,3,0,0,0)", false},
		{"0,{5},RobotMode()", 0, "5", "RobotMode()", false},
		{"garbage", 0, "", "", true},
	}
	for _, tc := range cases {
		got, err := parseDashResp(tc.in)
		if tc.wantParseErr {
			if err == nil {
				t.Errorf("parseDashResp(%q) expected error, got %+v", tc.in, got)
			}
			continue
		}
		if err != nil {
			t.Errorf("parseDashResp(%q): unexpected error %v", tc.in, err)
			continue
		}
		if got.ErrorID != tc.wantErr || got.Result != tc.wantResult || got.Command != tc.wantCommand {
			t.Errorf("parseDashResp(%q) = %+v; want err=%d result=%q cmd=%q",
				tc.in, got, tc.wantErr, tc.wantResult, tc.wantCommand)
		}
	}
}
