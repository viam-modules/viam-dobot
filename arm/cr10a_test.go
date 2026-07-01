package arm

import (
	"encoding/binary"
	"io"
	"math"
	"path/filepath"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
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
//
// The status bytes are written at LITERAL byte offsets (1025–1030), matching
// the documented V4.6 feedback layout, rather than via the offEnableStatus/
// offRunningStatus/offErrorStatus constants the parser reads back with. Using
// the same constants to write and read would make this test blind to an
// off-by-one in those constants. The immediate neighbor bytes (BrakeStatus,
// DragStatus, JogStatusCR) are deliberately set to the OPPOSITE value of the
// three fields under test, so any one-byte shift in either direction flips at
// least one asserted flag and fails the test loudly.
func TestFeedbackParse(t *testing.T) {
	// Guard: the parser's status-byte offsets must match the documented
	// V4.6 layout. If these drift, the literal-offset packet below would no
	// longer line up and the assertions would silently test the wrong bytes.
	if offEnableStatus != 1026 || offRunningStatus != 1028 || offErrorStatus != 1029 {
		t.Fatalf("status-byte offsets drifted from the documented V4.6 layout: enable=%d running=%d error=%d (want 1026/1028/1029)", offEnableStatus, offRunningStatus, offErrorStatus)
	}

	buf := make([]byte, feedbackPacketSize)

	// magic
	binary.LittleEndian.PutUint64(buf[offTestValue:], feedbackMagic)
	// robot mode = RobotModeRunning
	binary.LittleEndian.PutUint64(buf[offRobotMode:], uint64(RobotModeRunning))
	// Status bytes at LITERAL offsets from the V4.6 byte-position table.
	// Neighbors are set to the opposite value to catch off-by-one drift.
	buf[1025] = 0 // BrakeStatus   (neighbor below EnableStatus)
	buf[1026] = 1 // EnableStatus  (under test -> Enabled == true)
	buf[1027] = 0 // DragStatus    (neighbor between Enable and Running)
	buf[1028] = 1 // RunningStatus (under test -> Running == true)
	buf[1029] = 0 // ErrorStatus   (under test -> HasError == false)
	buf[1030] = 1 // JogStatusCR   (neighbor above ErrorStatus)

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
// joint velocity/accel limits (rad/s) to the controller's 1..100 VelJ/AccJ
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

// TestURDFParse loads the URDF directly (relative path works because go test
// sets the working directory to the package directory) and verifies it produces
// a valid 6-DoF model whose home pose is finite.
func TestURDFParse(t *testing.T) {
	model, err := referenceframe.ParseModelXMLFile("cr10a.urdf", "cr10a", nil)
	if err != nil {
		t.Fatalf("ParseModelXMLFile: %v", err)
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
	if math.IsNaN(pt.X) || math.IsNaN(pt.Y) || math.IsNaN(pt.Z) ||
		math.IsInf(pt.X, 0) || math.IsInf(pt.Y, 0) || math.IsInf(pt.Z, 0) {
		t.Fatalf("non-finite home pose %v", pt)
	}
}

// TestMakeModelFrameURDF exercises makeModelFrame's URDF branch end to end,
// including the per-mesh decimation-ratio default. This is the test that would
// have caught a mesh-count mismatch (one ratio per joint vs. per collision
// mesh). go test runs with CWD = the package dir (arm/), and makeModelFrame
// joins VIAM_MODULE_ROOT + "arm" + "cr10a.urdf", so ".." resolves to the repo
// root and the path lands back on ../arm/cr10a.urdf.
func TestMakeModelFrameURDF(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "..")
	model, err := makeModelFrame(&Config{Host: "x", UseURDF: true}, "cr10a")
	if err != nil {
		t.Fatalf("makeModelFrame URDF: %v", err)
	}
	if got := len(model.DoF()); got != 6 {
		t.Fatalf("expected 6 DoF, got %d", got)
	}
}

// TestMakeModelFrameJSON confirms the default (no use_urdf) branch still yields
// the embedded 6-DoF capsule model.
func TestMakeModelFrameJSON(t *testing.T) {
	model, err := makeModelFrame(&Config{Host: "x"}, "cr10a")
	if err != nil {
		t.Fatalf("makeModelFrame JSON: %v", err)
	}
	if got := len(model.DoF()); got != 6 {
		t.Fatalf("expected 6 DoF, got %d", got)
	}
}

// TestConfigValidateMeshDecimationRatios checks that out-of-range ratios are
// rejected and valid ratios (including the boundary values 0 and 1) are accepted.
func TestConfigValidateMeshDecimationRatios(t *testing.T) {
	good := &Config{Host: "1.2.3.4", MeshDecimationRatios: []float64{0, 0.5, 1}}
	if _, _, err := good.Validate("path"); err != nil {
		t.Fatalf("valid ratios rejected: %v", err)
	}
	bad := &Config{Host: "1.2.3.4", MeshDecimationRatios: []float64{1.5}}
	if _, _, err := bad.Validate("path"); err == nil {
		t.Fatalf("expected error for out-of-range ratio, got nil")
	}
	negative := &Config{Host: "1.2.3.4", MeshDecimationRatios: []float64{-0.1}}
	if _, _, err := negative.Validate("path"); err == nil {
		t.Fatalf("expected error for negative ratio, got nil")
	}
}

// TestJSONURDFForwardKinematicsAgree samples several joint configurations and
// asserts that the embedded JSON kinematics model and the URDF kinematics model
// produce tool poses that agree within a loose tolerance. A large deviation
// signals that the two independently-authored models disagree on the tool frame,
// which would cause a silent TCP jump when toggling use_urdf.
func TestJSONURDFForwardKinematicsAgree(t *testing.T) {
	jsonModel, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, "cr10a")
	if err != nil {
		t.Fatalf("UnmarshalModelJSON: %v", err)
	}
	urdfModel, err := referenceframe.ParseModelXMLFile("cr10a.urdf", "cr10a", nil)
	if err != nil {
		t.Fatalf("ParseModelXMLFile: %v", err)
	}

	// Representative joint configs (radians).
	configs := [][]referenceframe.Input{
		{0, 0, 0, 0, 0, 0},
		{0.5, -0.3, 0.7, 0.2, -0.4, 0.1},
		{-1.2, 0.6, -0.9, 1.0, 0.3, -0.7},
	}
	const posTolMM = 10.0 // loose: analytic links vs mesh origins
	const oriTolDeg = 2.0

	for _, c := range configs {
		jp, err := jsonModel.Transform(c)
		if err != nil {
			t.Fatalf("json Transform %v: %v", c, err)
		}
		up, err := urdfModel.Transform(c)
		if err != nil {
			t.Fatalf("urdf Transform %v: %v", c, err)
		}

		dist := jp.Point().Sub(up.Point()).Norm()
		oriDiff := spatialmath.QuatToR3AA(
			spatialmath.OrientationBetween(jp.Orientation(), up.Orientation()).Quaternion(),
		).Norm() * 180 / math.Pi

		t.Logf("config %v: posΔ=%.2fmm oriΔ=%.2f°", c, dist, oriDiff)
		if dist > posTolMM {
			t.Errorf("config %v: position diff %.2fmm exceeds %.1fmm", c, dist, posTolMM)
		}
		if oriDiff > oriTolDeg {
			t.Errorf("config %v: orientation diff %.2f° exceeds %.1f°", c, oriDiff, oriTolDeg)
		}
	}
}

// TestJSONURDFJointLimitsAgree asserts the embedded JSON kinematics model and the
// URDF model expose the same per-joint limits. The URDF (<limit lower/upper> in
// radians) is the source of truth; the JSON stores the same bounds in degrees.
// Because the JSON limits are hand-authored, a copy that drifts from the URDF
// (e.g. the old ±180°/±360° placeholders) lets the planner refuse or attempt
// joint angles the real arm's range disagrees with — this guards against that.
func TestJSONURDFJointLimitsAgree(t *testing.T) {
	jsonModel, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, "cr10a")
	if err != nil {
		t.Fatalf("UnmarshalModelJSON: %v", err)
	}
	urdfModel, err := referenceframe.ParseModelXMLFile("cr10a.urdf", "cr10a", nil)
	if err != nil {
		t.Fatalf("ParseModelXMLFile: %v", err)
	}

	jDoF := jsonModel.DoF()
	uDoF := urdfModel.DoF()
	if len(jDoF) != len(uDoF) {
		t.Fatalf("DoF mismatch: json %d, urdf %d", len(jDoF), len(uDoF))
	}

	// DoF() returns limits in radians. 1e-4 rad ≈ 0.006° absorbs the
	// deg→rad round-trip without hiding a real placeholder mismatch.
	const tolRad = 1e-4
	for i := range jDoF {
		if math.Abs(jDoF[i].Min-uDoF[i].Min) > tolRad || math.Abs(jDoF[i].Max-uDoF[i].Max) > tolRad {
			t.Errorf("joint %d limits disagree: json [%.5f, %.5f] rad, urdf [%.5f, %.5f] rad",
				i+1, jDoF[i].Min, jDoF[i].Max, uDoF[i].Min, uDoF[i].Max)
		}
	}
}

// TestParseDragSensitivityArgs covers the set_drag_sensitivity arg parsing:
// required value (1..90), optional index (default 0, else 0..6), float64→int
// truncation, and — critically — that the index-out-of-range failure returns a
// DISTINCT message from the value failure so callers can tell them apart.
func TestParseDragSensitivityArgs(t *testing.T) {
	const valueMsgSub = `"value"`
	const indexMsgSub = `"index"`

	cases := []struct {
		name      string
		cmd       map[string]interface{}
		wantIndex int
		wantValue int
		wantErr   bool
		// errMsgSub, if set, must be a substring of the returned error so we
		// can prove the value vs index failures are distinguishable.
		errMsgSub string
	}{
		{
			name:      "value only -> index defaults to 0",
			cmd:       map[string]interface{}{"value": float64(50)},
			wantIndex: 0,
			wantValue: 50,
		},
		{
			name:      "index and value valid",
			cmd:       map[string]interface{}{"index": float64(3), "value": float64(20)},
			wantIndex: 3,
			wantValue: 20,
		},
		{
			name:      "missing value -> value error",
			cmd:       map[string]interface{}{"index": float64(2)},
			wantErr:   true,
			errMsgSub: valueMsgSub,
		},
		{
			name:      "value 0 -> value error",
			cmd:       map[string]interface{}{"value": float64(0)},
			wantErr:   true,
			errMsgSub: valueMsgSub,
		},
		{
			name:      "value 100 -> value error",
			cmd:       map[string]interface{}{"value": float64(100)},
			wantErr:   true,
			errMsgSub: valueMsgSub,
		},
		{
			name:      "index 7 -> distinct index error",
			cmd:       map[string]interface{}{"index": float64(7), "value": float64(50)},
			wantErr:   true,
			errMsgSub: indexMsgSub,
		},
		{
			name:      "index -1 -> distinct index error",
			cmd:       map[string]interface{}{"index": float64(-1), "value": float64(50)},
			wantErr:   true,
			errMsgSub: indexMsgSub,
		},
		{
			// Truncation toward zero is intentional and matches set_speed's
			// int(v): 1.9 becomes 1 (valid), not 2.
			name:      "fractional value truncates toward zero",
			cmd:       map[string]interface{}{"value": float64(1.9)},
			wantIndex: 0,
			wantValue: 1,
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			index, value, err := parseDragSensitivityArgs(tc.cmd)
			if tc.wantErr {
				if err == nil {
					t.Fatalf("expected error, got index=%d value=%d nil err", index, value)
				}
				if tc.errMsgSub != "" && !strings.Contains(err.Error(), tc.errMsgSub) {
					t.Errorf("error %q does not contain %q (value/index messages must be distinguishable)", err.Error(), tc.errMsgSub)
				}
				return
			}
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if index != tc.wantIndex || value != tc.wantValue {
				t.Errorf("got (index=%d, value=%d), want (index=%d, value=%d)", index, value, tc.wantIndex, tc.wantValue)
			}
		})
	}

	// Sanity: the value and index failures really do produce different text.
	_, _, valErr := parseDragSensitivityArgs(map[string]interface{}{"value": float64(0)})
	_, _, idxErr := parseDragSensitivityArgs(map[string]interface{}{"value": float64(50), "index": float64(9)})
	if valErr == nil || idxErr == nil || valErr.Error() == idxErr.Error() {
		t.Fatalf("value error %v and index error %v must be non-nil and distinct", valErr, idxErr)
	}
}

// cr10aWorldPosesZero reconstructs world-frame poses for every link and joint
// frame in cfg, evaluated at all-zero inputs (home position). Results are keyed
// by frame ID and memoized for efficiency.
func cr10aWorldPosesZero(t *testing.T, cfg *referenceframe.ModelConfigJSON) map[string]spatialmath.Pose {
	t.Helper()
	transforms := map[string]referenceframe.Frame{}
	parent := map[string]string{}
	for i := range cfg.Links {
		l := &cfg.Links[i]
		lif, err := l.ParseConfig()
		if err != nil {
			t.Fatalf("link %s ParseConfig: %v", l.ID, err)
		}
		f, err := lif.ToStaticFrame(l.ID)
		if err != nil {
			t.Fatalf("link %s ToStaticFrame: %v", l.ID, err)
		}
		transforms[l.ID] = f
		parent[l.ID] = l.Parent
	}
	for i := range cfg.Joints {
		j := &cfg.Joints[i]
		f, err := j.ToFrame()
		if err != nil {
			t.Fatalf("joint %s ToFrame: %v", j.ID, err)
		}
		transforms[j.ID] = f
		parent[j.ID] = j.Parent
	}
	memo := map[string]spatialmath.Pose{}
	var world func(name string) spatialmath.Pose
	world = func(name string) spatialmath.Pose {
		if name == "" || name == referenceframe.World {
			return spatialmath.NewZeroPose()
		}
		if p, ok := memo[name]; ok {
			return p
		}
		fr, ok := transforms[name]
		if !ok {
			t.Fatalf("frame %q not found", name)
		}
		var in []referenceframe.Input
		if d := len(fr.DoF()); d > 0 {
			in = make([]referenceframe.Input, d)
		}
		local, err := fr.Transform(in)
		if err != nil {
			t.Fatalf("frame %s Transform: %v", name, err)
		}
		res := spatialmath.Compose(world(parent[name]), local)
		memo[name] = res
		return res
	}
	for name := range transforms {
		world(name)
	}
	return memo
}

// TestPerLinkFrameAlignment verifies that the JSON (UR-derived) and URDF link
// frames coincide at home position to within 1 mm / 1°. This is the guard that
// makes it safe to key the bundled STL meshes by either model's frame names —
// if the frames ever drift, this test fails before Get3DModels silently
// misregisters a mesh.
func TestPerLinkFrameAlignment(t *testing.T) {
	jsonModel, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, "cr10a")
	if err != nil {
		t.Fatal(err)
	}
	urdfModel, err := referenceframe.ParseModelXMLFile("cr10a.urdf", "cr10a", nil)
	if err != nil {
		t.Fatal(err)
	}
	jp := cr10aWorldPosesZero(t, jsonModel.ModelConfig())
	up := cr10aWorldPosesZero(t, urdfModel.ModelConfig())

	pairs := [][2]string{
		{"base_link", "base_link"},
		{"shoulder_link", "Link1"},
		{"upper_arm_link", "Link2"},
		{"forearm_link", "Link3"},
		{"wrist_1_link", "Link4"},
		{"wrist_2_link", "Link5"},
		{"ee_link", "Link6"},
	}
	const posTolMM = 1.0
	const oriTolDeg = 1.0
	for _, pr := range pairs {
		j, ok1 := jp[pr[0]]
		u, ok2 := up[pr[1]]
		if !ok1 || !ok2 {
			t.Fatalf("missing pose json[%s]=%v urdf[%s]=%v", pr[0], ok1, pr[1], ok2)
		}
		posD := j.Point().Sub(u.Point()).Norm()
		oriD := spatialmath.QuatToR3AA(
			spatialmath.OrientationBetween(j.Orientation(), u.Orientation()).Quaternion(),
		).Norm() * 180 / math.Pi
		t.Logf("%s<->%s posΔ=%.4fmm oriΔ=%.4f°", pr[0], pr[1], posD, oriD)
		if posD > posTolMM {
			t.Errorf("%s<->%s position diff %.4fmm exceeds %.1fmm", pr[0], pr[1], posD, posTolMM)
		}
		if oriD > oriTolDeg {
			t.Errorf("%s<->%s orientation diff %.4f° exceeds %.1f°", pr[0], pr[1], oriD, oriTolDeg)
		}
	}
}

// TestJSONGeometriesFitMeshes guards that each JSON collision volume is centered
// on AND aligned with the link mesh it approximates. Every STL is authored in its
// link's frame (the JSON link frame it maps to via cr10aMeshParts), and the GLB
// served to the viewer renders in that same frame, so the collision geometry must
// share it. Two failure modes have bitten this file: capsules copied from the UR
// model sat up to ~200 mm off their meshes (center), and capsules whose long axis
// pointed along the wrong local axis rendered perpendicular to the link
// (orientation). This checks both.
func TestJSONGeometriesFitMeshes(t *testing.T) {
	model, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, "cr10a")
	if err != nil {
		t.Fatalf("UnmarshalModelJSON: %v", err)
	}
	geomCfg := map[string]*spatialmath.GeometryConfig{}
	for _, l := range model.ModelConfig().Links {
		if l.Geometry != nil {
			geomCfg[l.ID] = l.Geometry
		}
	}

	const centerTolMM = 5.0
	const sizeTolMM = 20.0 // capsule/box extent vs mesh AABB extent (approximation slack)
	for _, part := range cr10aMeshParts {
		mesh, err := spatialmath.NewMeshFromSTLFile(filepath.Join("meshes", "cr10", part.stlFile))
		if err != nil {
			t.Fatalf("load %s: %v", part.stlFile, err)
		}
		lo := r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
		hi := r3.Vector{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
		for _, tri := range mesh.Triangles() {
			for _, p := range tri.Points() {
				lo = r3.Vector{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
				hi = r3.Vector{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
			}
		}
		center := lo.Add(hi).Mul(0.5)
		ext := hi.Sub(lo)
		extArr := [3]float64{ext.X, ext.Y, ext.Z}
		longAxis := 0
		for i, e := range extArr {
			if e > extArr[longAxis] {
				longAxis = i
			}
		}

		cfg, ok := geomCfg[part.jsonName]
		if !ok {
			t.Errorf("link %q has no collision geometry", part.jsonName)
			continue
		}

		// Center: geometry offset vs mesh AABB center, both in the link frame.
		if d := center.Sub(cfg.TranslationOffset).Norm(); d > centerTolMM {
			t.Errorf("link %q geometry center %.1f is %.1fmm from mesh center %.1f",
				part.jsonName, cfg.TranslationOffset, d, center)
		}

		geom, err := cfg.ParseConfig()
		if err != nil {
			t.Fatalf("link %q ParseConfig: %v", part.jsonName, err)
		}

		if cfg.L > 0 { // capsule: verify long axis points along the mesh's longest AABB axis
			ov := geom.Pose().Orientation().OrientationVectorRadians()
			capAxis := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}
			card := r3.Vector{}
			switch longAxis {
			case 0:
				card = r3.Vector{X: 1}
			case 1:
				card = r3.Vector{Y: 1}
			case 2:
				card = r3.Vector{Z: 1}
			}
			dot := math.Abs(capAxis.Dot(card))
			t.Logf("%-16s capsule axis=%.2f mesh long axis=%v |dot|=%.3f l=%.1f (ext %.1f)",
				part.jsonName, capAxis, card, dot, cfg.L, extArr[longAxis])
			if dot < 0.99 {
				t.Errorf("link %q capsule long axis %.2f is not aligned with mesh longest axis %v (|dot|=%.3f)",
					part.jsonName, capAxis, card, dot)
			}
			if math.Abs(cfg.L-extArr[longAxis]) > sizeTolMM {
				t.Errorf("link %q capsule length %.1f differs from mesh long extent %.1f by >%.0fmm",
					part.jsonName, cfg.L, extArr[longAxis], sizeTolMM)
			}
		} else { // box: dims should match the AABB extents per axis
			for i, d := range [3]float64{cfg.X, cfg.Y, cfg.Z} {
				if math.Abs(d-extArr[i]) > sizeTolMM {
					t.Errorf("link %q box dim[%d]=%.1f differs from mesh extent %.1f by >%.0fmm",
						part.jsonName, i, d, extArr[i], sizeTolMM)
				}
			}
			t.Logf("%-16s box dims=(%.1f,%.1f,%.1f) mesh ext=(%.1f,%.1f,%.1f)",
				part.jsonName, cfg.X, cfg.Y, cfg.Z, ext.X, ext.Y, ext.Z)
		}
	}
}

// TestGet3DModelsReturnsMeshes exercises Get3DModels in both kinematics modes by
// pointing VIAM_MODULE_ROOT at the repo root (go test runs with CWD = arm/, so
// ".." resolves there). Each case asserts 7 GLB entries keyed by the mode's
// frame names, with non-empty mesh bytes and a valid glTF binary header.
func TestGet3DModelsReturnsMeshes(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "..")

	cases := []struct {
		name      string
		useURDF   bool
		wantNames []string
	}{
		{
			name:      "json frame names (use_urdf false)",
			useURDF:   false,
			wantNames: []string{"base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "ee_link"},
		},
		{
			name:      "urdf frame names (use_urdf true)",
			useURDF:   true,
			wantNames: []string{"base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"},
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			a := &cr10a{
				logger:      logging.NewTestLogger(t),
				meshUseURDF: tc.useURDF,
			}

			meshes, err := a.Get3DModels(t.Context(), nil)
			if err != nil {
				t.Fatalf("Get3DModels returned error: %v", err)
			}
			if len(meshes) != len(tc.wantNames) {
				t.Fatalf("expected %d meshes, got %d (keys %v)", len(tc.wantNames), len(meshes), keysOf(meshes))
			}
			for _, name := range tc.wantNames {
				m, ok := meshes[name]
				if !ok {
					t.Errorf("missing mesh for frame %q", name)
					continue
				}
				if m.ContentType != "model/gltf-binary" {
					t.Errorf("frame %q: expected ContentType %q, got %q", name, "model/gltf-binary", m.ContentType)
				}
				if len(m.Mesh) == 0 {
					t.Errorf("frame %q: Mesh bytes are empty", name)
				} else if string(m.Mesh[:4]) != "glTF" {
					t.Errorf("frame %q: expected glTF binary magic, got % x", name, m.Mesh[:min(4, len(m.Mesh))])
				}
			}
		})
	}
}

func keysOf(m map[string]*commonpb.Mesh) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}
