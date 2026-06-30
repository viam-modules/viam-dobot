// Package arm implements the Dobot CR10A as a Viam arm component.
//
// Architecture:
//   - dashClient (port 29999): synchronous ASCII command/ack channel.
//   - feedbackClient (port 30004): async 125 Hz status broadcast reader.
//   - cr10a wraps both behind the arm.Arm interface and consults a kinematic
//     model loaded from cr10a_kinematics.json (embedded via go:embed).
//
// The Dobot wire protocol uses millimeters and degrees everywhere; Viam uses
// millimeters and radians. All conversions happen at this boundary —
// downstream callers only ever see SI units / referenceframe.Input radians.
package arm

import (
	"context"
	_ "embed"
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	commonpb "go.viam.com/api/common/v1"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

// Model identifier registered with the framework.
//
// Update the namespace if you fork this module under a different organization;
// the wire model string is `<namespace>:dobot:cr10a`.
var Model = resource.ModelNamespace("viam-soleng").WithFamily("dobot").WithModel("cr10a")

//go:embed cr10a_kinematics.json
var cr10aKinematicsJSON []byte

// Defaults that match Dobot's documented safe values.
const (
	defaultSpeedFactor = 50 // global SpeedFactor() percent (1..100)
	defaultJointSpeed  = 50 // VelJ() percent (1..100)
	defaultJointAccel  = 50 // AccJ() percent (1..100)

	// motionPollInterval is how often we re-check feedback while a motion is
	// in flight. The CR controller emits feedback at 125 Hz, so 25ms is plenty.
	motionPollInterval = 25 * time.Millisecond

	// motionStartGrace is how long we wait after issuing a move before we
	// start interpreting "Running == false" as "motion complete". Without it
	// we'd race the controller — it takes a few ms to start executing.
	motionStartGrace = 200 * time.Millisecond

	// jointTolerance is the per-joint slop we accept when checking whether a
	// completed move actually reached its target. Degrees.
	jointToleranceDeg = 0.5

	// maxJointSpeedDegPerSec is the controller's nominal 100% joint velocity in
	// degrees/sec. Used to convert MoveOptions.MaxVelRads into the VelJ 1..100
	// percent the wire protocol takes. The CR10A datasheet quotes per-joint
	// maxes in the 180–225 °/s range; 180 is the conservative floor across
	// joints. If you tune the controller for higher max speed and the planner
	// is consistently asking for slower moves than it should, raise this.
	maxJointSpeedDegPerSec = 180.0

	// maxJointAccelDegPerSec2 plays the same role for AccJ. The CR10A datasheet
	// doesn't publish an authoritative absolute max joint accel; 180 °/s² is a
	// pragmatic default that makes percent-of-max conversions sane.
	maxJointAccelDegPerSec2 = 180.0
)

// Config is the JSON config block for a CR10A arm.
type Config struct {
	// Host is the controller IP address (required).
	Host string `json:"host"`
	// DashboardPort is the command port; defaults to 29999.
	DashboardPort int `json:"dashboard_port,omitempty"`
	// FeedbackPort is the realtime broadcast port; defaults to 30004.
	FeedbackPort int `json:"feedback_port,omitempty"`
	// SpeedFactor is the global speed override applied at startup, 1..100.
	SpeedFactor int `json:"speed_factor,omitempty"`
	// JointSpeed is the per-MovJ speed ratio, 1..100.
	JointSpeed int `json:"joint_speed,omitempty"`
	// JointAccel is the per-MovJ acceleration ratio, 1..100.
	JointAccel int `json:"joint_accel,omitempty"`
	// AutoEnable enables servos at module start; default true.
	AutoEnable *bool `json:"auto_enable,omitempty"`
	// UseURDF loads kinematics + meshes from arm/cr10a.urdf instead of the
	// embedded capsule JSON. Default false (embedded JSON) until the URDF is
	// hardware-validated. Requires VIAM_MODULE_ROOT to be set (it is, under
	// viam-server).
	UseURDF bool `json:"use_urdf,omitempty"`
	// MeshDecimationRatios is the per-collision-mesh simplification ratio in
	// [0,1] used when UseURDF is set. The RDK URDF parser applies these to
	// collision meshes in URDF document order — for the CR10A that's base_link
	// followed by the 6 link meshes (7 total), not one per joint. Only ratios
	// strictly in (0,1) actually decimate a mesh; a value of 0 or 1 leaves that
	// mesh at full resolution. Lower = more aggressive within (0,1). If fewer
	// than 7 ratios are supplied, trailing meshes are left undecimated. Defaults
	// to 0.1 for each of the 7 meshes when empty. Ignored unless UseURDF is true.
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`
}

// Validate satisfies resource.ConfigValidator.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	for name, v := range map[string]int{
		"speed_factor": cfg.SpeedFactor,
		"joint_speed":  cfg.JointSpeed,
		"joint_accel":  cfg.JointAccel,
	} {
		if v < 0 || v > 100 {
			return nil, nil, fmt.Errorf("%s must be between 1 and 100, got %d", name, v)
		}
	}
	for i, r := range cfg.MeshDecimationRatios {
		if math.IsNaN(r) || r < 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratios[%d] must be in [0, 1], got %f", i, r)
		}
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(arm.API, Model, resource.Registration[arm.Arm, *Config]{
		Constructor: newCR10A,
	})
}

// cr10aMeshSTLFiles are the bundled link meshes in kinematic order.
var cr10aMeshSTLFiles = []string{
	"base_link.STL", "Link1.STL", "Link2.STL", "Link3.STL", "Link4.STL", "Link5.STL", "Link6.STL",
}

// cr10aJSONFrameNames and cr10aURDFFrameNames are the active model's frame
// names for each mesh in cr10aMeshSTLFiles, in the same order. The meshes are
// authored in the URDF link frames; the JSON (UR-derived) link frames coincide
// with them (verified by TestPerLinkFrameAlignment), so keying by either is
// correct.
var cr10aJSONFrameNames = []string{
	"base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "ee_link",
}
var cr10aURDFFrameNames = []string{
	"base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6",
}

// cr10a is the live arm.Arm implementation.
type cr10a struct {
	resource.Named
	logger logging.Logger
	opMgr  *operation.SingleOperationManager

	mu       sync.RWMutex
	dash     *dashClient
	feedback *feedbackClient
	model    referenceframe.Model

	speedFactor   int
	jointSpeed    int
	jointAccel    int
	meshPartNames []string                  // active frame-name slice for Get3DModels
	meshCache     map[string]*commonpb.Mesh // lazily built; nil means not yet built
	meshCacheKey  string                    // strings.Join(meshPartNames,",") at cache-build time
}

func newCR10A(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (arm.Arm, error) {
	a := &cr10a{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		opMgr:  operation.NewSingleOperationManager(),
	}
	if err := a.Reconfigure(ctx, deps, conf); err != nil {
		return nil, err
	}
	return a, nil
}

// Reconfigure applies a new config in place, replacing the underlying TCP
// clients if connection parameters changed.
func (a *cr10a) Reconfigure(ctx context.Context, _ resource.Dependencies, conf resource.Config) error {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}

	model, err := makeModelFrame(newConf, conf.ResourceName().ShortName())
	if err != nil {
		return fmt.Errorf("loading CR10A kinematics: %w", err)
	}

	dashPort := orDefault(newConf.DashboardPort, defaultDashboardPort)
	fbPort := orDefault(newConf.FeedbackPort, defaultFeedbackPort)
	speedFactor := orDefault(newConf.SpeedFactor, defaultSpeedFactor)
	jointSpeed := orDefault(newConf.JointSpeed, defaultJointSpeed)
	jointAccel := orDefault(newConf.JointAccel, defaultJointAccel)
	autoEnable := true
	if newConf.AutoEnable != nil {
		autoEnable = *newConf.AutoEnable
	}

	// Determine which frame-name set to use for mesh keying.
	meshPartNames := cr10aJSONFrameNames
	if newConf.UseURDF {
		meshPartNames = cr10aURDFFrameNames
	}

	a.mu.Lock()
	// tear down old clients if any
	if a.dash != nil {
		_ = a.dash.close()
	}
	if a.feedback != nil {
		a.feedback.stop()
	}

	a.dash = newDashClient(newConf.Host, dashPort)
	a.feedback = newFeedbackClient(newConf.Host, fbPort)
	a.model = model
	a.speedFactor = speedFactor
	a.jointSpeed = jointSpeed
	a.jointAccel = jointAccel
	a.meshPartNames = meshPartNames
	// Invalidate the mesh cache whenever we reconfigure (the active name set may
	// have changed if use_urdf was toggled).
	a.meshCache = nil
	a.meshCacheKey = ""
	a.mu.Unlock()

	// start feedback reader (it survives Reconfigure because we just reassigned)
	a.feedback.start(context.Background())

	// open dashboard connection
	if err := a.dash.connect(ctx); err != nil {
		return fmt.Errorf("connecting to CR10A dashboard: %w", err)
	}

	// best-effort startup: clear any latched error, push speed factors, enable.
	if err := a.dash.clearError(ctx); err != nil {
		a.logger.CWarnw(ctx, "ClearError failed at startup", "err", err)
	}
	if err := a.dash.speedFactor(ctx, speedFactor); err != nil {
		a.logger.CWarnw(ctx, "SpeedFactor failed", "err", err)
	}
	if err := a.dash.velJ(ctx, jointSpeed); err != nil {
		a.logger.CWarnw(ctx, "VelJ failed", "err", err)
	}
	if err := a.dash.accJ(ctx, jointAccel); err != nil {
		a.logger.CWarnw(ctx, "AccJ failed", "err", err)
	}
	if autoEnable {
		if err := a.dash.enableRobot(ctx); err != nil {
			return fmt.Errorf("enabling CR10A: %w", err)
		}
	}

	// wait for the first feedback frame so subsequent calls don't race.
	waitCtx, cancel := context.WithTimeout(ctx, 3*time.Second)
	defer cancel()
	if _, err := a.feedback.waitForFrame(waitCtx); err != nil {
		a.logger.CWarnw(ctx, "no initial feedback frame received", "err", err)
	}

	return nil
}

// Close shuts down both TCP clients. It does NOT disable the servos —
// callers who want to power down should issue DoCommand{"disable": true}.
func (a *cr10a) Close(ctx context.Context) error {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.feedback != nil {
		a.feedback.stop()
	}
	if a.dash != nil {
		return a.dash.close()
	}
	return nil
}

// ---------- arm.Arm: pose / inputs ----------

func (a *cr10a) JointPositions(ctx context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
	fr, err := a.latestFrame(ctx)
	if err != nil {
		return nil, err
	}
	out := make([]referenceframe.Input, 6)
	for i := 0; i < 6; i++ {
		// referenceframe.Input is a float64 alias; revolute inputs are radians.
		out[i] = rdkutils.DegToRad(fr.JointDegs[i])
	}
	return out, nil
}

func (a *cr10a) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return a.JointPositions(ctx, nil)
}

func (a *cr10a) EndPosition(ctx context.Context, _ map[string]interface{}) (spatialmath.Pose, error) {
	a.mu.RLock()
	model := a.model
	a.mu.RUnlock()
	inputs, err := a.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return model.Transform(inputs)
}

func (a *cr10a) Kinematics(_ context.Context) (referenceframe.Model, error) {
	a.mu.RLock()
	defer a.mu.RUnlock()
	return a.model, nil
}

func (a *cr10a) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := a.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	a.mu.RLock()
	model := a.model
	a.mu.RUnlock()
	gif, err := model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

// Get3DModels returns the bundled link meshes (base_link + Link1..Link6) as
// PLY, keyed by the active kinematic model's frame names. The active name set
// is cr10aJSONFrameNames when use_urdf is false and cr10aURDFFrameNames when
// use_urdf is true. The frames coincide geometrically (verified by
// TestPerLinkFrameAlignment), so the same seven STL files serve both models.
//
// If VIAM_MODULE_ROOT is unset the method logs a warning and returns an empty
// map with a nil error — meshes are simply unavailable in that environment
// (unit tests, developer workstations) and 3D visualisation should degrade
// gracefully. A per-file read/parse failure returns a wrapped error.
//
// Results are cached on the struct and invalidated by Reconfigure.
func (a *cr10a) Get3DModels(_ context.Context, _ map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		a.logger.Warn("Get3DModels: VIAM_MODULE_ROOT is not set; returning empty mesh map")
		return map[string]*commonpb.Mesh{}, nil
	}

	a.mu.RLock()
	names := a.meshPartNames
	cacheKey := a.meshCacheKey
	cached := a.meshCache
	a.mu.RUnlock()

	// Key the cache on the full active name set so a use_urdf toggle (which
	// swaps the slice but keeps base_link as the first element) actually
	// invalidates a stale cache. The two slices differ from Link1 onward.
	activeKey := strings.Join(names, ",")
	if cached != nil && cacheKey == activeKey {
		return cached, nil
	}

	built, err := buildMeshMap(root, names)
	if err != nil {
		return nil, err
	}

	a.mu.Lock()
	a.meshCache = built
	a.meshCacheKey = activeKey
	a.mu.Unlock()

	return built, nil
}

// buildMeshMap reads each STL under moduleRoot/arm/meshes/cr10/, converts it
// to PLY via TrianglesToPLYBytes, and keys the result by the matching entry in
// names. names and cr10aMeshSTLFiles must be the same length.
func buildMeshMap(moduleRoot string, names []string) (map[string]*commonpb.Mesh, error) {
	out := make(map[string]*commonpb.Mesh, len(cr10aMeshSTLFiles))
	for i, stlFile := range cr10aMeshSTLFiles {
		path := filepath.Join(moduleRoot, "arm", "meshes", "cr10", stlFile)
		mesh, err := spatialmath.NewMeshFromSTLFile(path)
		if err != nil {
			return nil, fmt.Errorf("loading mesh %q: %w", path, err)
		}
		out[names[i]] = &commonpb.Mesh{
			ContentType: "ply",
			Mesh:        mesh.TrianglesToPLYBytes(false),
		}
	}
	return out, nil
}

// ---------- arm.Arm: motion ----------

func (a *cr10a) MoveToPosition(ctx context.Context, pos spatialmath.Pose, _ map[string]interface{}) error {
	ctx, done := a.opMgr.New(ctx)
	defer done()
	// Delegate to the planner: it computes a joint trajectory using our
	// Kinematics() and ultimately calls MoveToJointPositions. We never send
	// MovL/MovJ Cartesian commands directly — the kinematic model is the
	// source of truth, not the device's reported TCP.
	return armplanning.MoveArm(ctx, a.logger, a, pos)
}

func (a *cr10a) MoveToJointPositions(ctx context.Context, joints []referenceframe.Input, _ map[string]interface{}) error {
	return a.moveJoint(ctx, joints, nil)
}

func (a *cr10a) MoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	opts *arm.MoveOptions,
	_ map[string]interface{},
) error {
	for i, goal := range positions {
		if err := a.moveJoint(ctx, goal, opts); err != nil {
			return fmt.Errorf("waypoint %d: %w", i, err)
		}
	}
	return nil
}

func (a *cr10a) GoToInputs(ctx context.Context, steps ...[]referenceframe.Input) error {
	return a.MoveThroughJointPositions(ctx, steps, nil, nil)
}

// moveJoint is the single place that issues a `MovJ(joint=…)`. Callers may supply
// MoveOptions to override speed/accel for this move; nil falls back to the
// values set in Reconfigure. The RLock is held through the wire calls and the
// completion poll so a concurrent Close/Reconfigure can't yank the dash client
// from under the in-flight write.
func (a *cr10a) moveJoint(ctx context.Context, joints []referenceframe.Input, opts *arm.MoveOptions) error {
	if err := arm.CheckDesiredJointPositions(ctx, a, joints); err != nil {
		return err
	}
	if len(joints) != 6 {
		return fmt.Errorf("expected 6 joint inputs, got %d", len(joints))
	}
	ctx, done := a.opMgr.New(ctx)
	defer done()

	var degTarget [6]float64
	for i := 0; i < 6; i++ {
		degTarget[i] = rdkutils.RadToDeg(joints[i])
	}

	a.mu.RLock()
	defer a.mu.RUnlock()
	if a.dash == nil {
		return errors.New("CR10A dashboard client not initialized")
	}

	speedPct := a.jointSpeed
	accelPct := a.jointAccel
	if opts != nil {
		if opts.MaxVelRads > 0 {
			speedPct = radPerSecToPercent(opts.MaxVelRads, maxJointSpeedDegPerSec)
		}
		if opts.MaxAccRads > 0 {
			accelPct = radPerSecToPercent(opts.MaxAccRads, maxJointAccelDegPerSec2)
		}
	}
	if err := a.dash.velJ(ctx, speedPct); err != nil {
		return fmt.Errorf("VelJ: %w", err)
	}
	if err := a.dash.accJ(ctx, accelPct); err != nil {
		return fmt.Errorf("AccJ: %w", err)
	}
	if err := a.dash.jointMovJ(ctx, degTarget); err != nil {
		return fmt.Errorf("MovJ: %w", err)
	}
	return a.waitForMotionCompleteLocked(ctx, degTarget)
}

// Stop immediately halts queued motion. The servos remain enabled; the arm
// can resume motion with the next MoveTo* call.
func (a *cr10a) Stop(ctx context.Context, _ map[string]interface{}) error {
	a.opMgr.CancelRunning(ctx)
	a.mu.RLock()
	defer a.mu.RUnlock()
	if a.dash == nil {
		return errors.New("CR10A dashboard client not initialized")
	}
	return a.dash.stop(ctx)
}

// IsMoving consults the feedback packet's running_status byte. Falls back to
// the operation manager if no live frame is available — either because none
// has been seen yet or because the feedback connection has been lost.
func (a *cr10a) IsMoving(ctx context.Context) (bool, error) {
	fr, ok, _ := a.feedback.latest()
	if !ok {
		return a.opMgr.OpRunning(), nil
	}
	return fr.Running || fr.Mode == RobotModeRunning, nil
}

// DoCommand exposes Dobot-specific actions that don't fit the arm.Arm API.
//
// Supported commands:
//   - {"action": "enable"}        → EnableRobot()
//   - {"action": "disable"}       → DisableRobot()
//   - {"action": "clear_error"}   → ClearError()
//   - {"action": "emergency_stop"}→ EmergencyStop(1)
//   - {"action": "set_speed", "value": 1..100} → SpeedFactor(value), persisted
//   - {"action": "robot_mode"}    → returns {"mode": <int>}
//   - {"action": "start_drag"}    → StartDrag() (enter drag/freedrive; refused
//     while an alarm is latched — clear_error first)
//   - {"action": "stop_drag"}     → StopDrag()
//   - {"action": "set_drag_sensitivity", "value": 1..90, "index": 0..6} →
//     DragSensivity(index,value); index 0 = all axes (default), 1..6 = J1..J6;
//     smaller value = more resistance
func (a *cr10a) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if a.dash == nil {
		return nil, errors.New("CR10A dashboard client not initialized")
	}
	action, _ := cmd["action"].(string)
	switch action {
	case "enable":
		return map[string]interface{}{"ok": true}, a.dash.enableRobot(ctx)
	case "disable":
		return map[string]interface{}{"ok": true}, a.dash.disableRobot(ctx)
	case "clear_error":
		return map[string]interface{}{"ok": true}, a.dash.clearError(ctx)
	case "emergency_stop":
		return map[string]interface{}{"ok": true}, a.dash.emergencyStop(ctx)
	case "set_speed":
		// The override lives on the controller; Reconfigure intentionally
		// re-pushes the configured a.speedFactor on every reload, so a runtime
		// set_speed is transient (matches xArm's pattern, which rebuilds on
		// any config change).
		v, ok := cmd["value"].(float64) // JSON numbers come in as float64
		if !ok {
			return nil, errors.New(`set_speed requires "value" int 1..100`)
		}
		return map[string]interface{}{"ok": true}, a.dash.speedFactor(ctx, int(v))
	case "robot_mode":
		mode, err := a.dash.robotMode(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"mode": mode}, nil
	case "start_drag":
		return map[string]interface{}{"ok": true}, a.dash.startDrag(ctx)
	case "stop_drag":
		return map[string]interface{}{"ok": true}, a.dash.stopDrag(ctx)
	case "set_drag_sensitivity":
		// Unlike set_speed (which silently clamps), this rejects out-of-range
		// input — a deliberate UX choice so a bad sensitivity is surfaced rather
		// than quietly coerced. The wire-level dragSensivity clamp remains as a
		// defensive invariant.
		index, value, err := parseDragSensitivityArgs(cmd)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"ok": true}, a.dash.dragSensivity(ctx, index, value)
	default:
		return nil, fmt.Errorf("unknown action %q", action)
	}
}

// ---------- helpers ----------

// parseDragSensitivityArgs validates the set_drag_sensitivity DoCommand args.
// value is required and must be 1..90; index is optional (missing/non-numeric
// defaults to 0 = all axes) and, when present, must be 0..6. JSON numbers
// arrive as float64; the int() truncation toward zero is intentional and
// mirrors the set_speed handler (e.g. 1.9 → 1). The value and index failures
// return distinct messages so callers can tell which argument was wrong.
func parseDragSensitivityArgs(cmd map[string]interface{}) (index, value int, err error) {
	v, ok := cmd["value"].(float64) // JSON numbers come in as float64
	if !ok {
		return 0, 0, errors.New(`set_drag_sensitivity requires "value" int 1..90`)
	}
	value = int(v)
	if value < 1 || value > 90 {
		return 0, 0, errors.New(`set_drag_sensitivity requires "value" int 1..90`)
	}
	// index is optional; missing/non-numeric means 0 (all axes).
	if iv, ok := cmd["index"].(float64); ok {
		index = int(iv)
		if index < 0 || index > 6 {
			return 0, 0, errors.New(`set_drag_sensitivity "index" must be int 0..6`)
		}
	}
	return index, value, nil
}

// latestFrame fetches the latest feedback frame, blocking briefly if none has
// arrived yet (typical right after Reconfigure).
func (a *cr10a) latestFrame(ctx context.Context) (feedbackFrame, error) {
	fr, ok, err := a.feedback.latest()
	if ok {
		return fr, nil
	}
	if err != nil && ctx.Err() == nil {
		// known stale error — try to wait for a fresh frame anyway
	}
	return a.feedback.waitForFrame(ctx)
}

// waitForMotionCompleteLocked polls feedback until the controller signals it
// is no longer running and the joint positions are within tolerance of the
// target. Returns an error if the controller latched an alarm during the move.
//
// The caller MUST hold a.mu.RLock — moveJoint takes it across the `MovJ(joint=…)`
// and the entire wait, so a.dash is stable for the duration.
func (a *cr10a) waitForMotionCompleteLocked(ctx context.Context, targetDeg [6]float64) error {
	startCtx, cancelGrace := context.WithTimeout(ctx, motionStartGrace)
	defer cancelGrace()
	// Wait either for Running=true (motion started) or the grace window.
	for {
		fr, ok, _ := a.feedback.latest()
		if ok && (fr.Running || fr.Mode == RobotModeRunning) {
			break
		}
		if ok && fr.HasError {
			return errors.New("CR10A reported an error after motion command — check controller alarms (DoCommand clear_error to reset)")
		}
		select {
		case <-startCtx.Done():
			// no transition seen — continue to the completion poll, which
			// also handles the case of zero-distance moves.
			goto completed
		case <-time.After(motionPollInterval):
		}
	}
completed:

	for {
		select {
		case <-ctx.Done():
			// Caller cancelled — halt the arm before propagating the error.
			// Stop() also fires Stop() on the wire when invoked directly, so
			// when a user calls Stop() we may double-send; the controller
			// errors on the second call but the arm is correctly halted.
			stopCtx, cancelStop := context.WithTimeout(context.Background(), 2*time.Second)
			_ = a.dash.stop(stopCtx)
			cancelStop()
			return ctx.Err()
		case <-time.After(motionPollInterval):
		}

		fr, ok, _ := a.feedback.latest()
		if !ok {
			continue
		}
		if fr.HasError {
			return errors.New("CR10A latched an alarm during motion — call DoCommand clear_error to reset")
		}
		if !fr.Running && fr.Mode != RobotModeRunning {
			// Motion done. Sanity-check we reached the target.
			if jointsClose(fr.JointDegs, targetDeg, jointToleranceDeg) {
				return nil
			}
			// Sometimes the controller drops out of Running for a tick
			// between queued waypoints; allow one more poll.
			time.Sleep(motionPollInterval)
			fr2, ok2, _ := a.feedback.latest()
			if ok2 && !fr2.Running && fr2.Mode != RobotModeRunning {
				if jointsClose(fr2.JointDegs, targetDeg, jointToleranceDeg) {
					return nil
				}
				return fmt.Errorf("motion completed but joints did not reach target (got %v want %v)", fr2.JointDegs, targetDeg)
			}
		}
	}
}

// radPerSecToPercent maps a velocity/acceleration in radians/sec (or rad/s²)
// to the 1..100 VelJ/AccJ percent the controller takes, given an absolute
// max in degrees/sec for "100%".
func radPerSecToPercent(radPerSec, maxDegPerSec float64) int {
	deg := rdkutils.RadToDeg(radPerSec)
	pct := int(math.Round(deg / maxDegPerSec * 100))
	if pct < 1 {
		return 1
	}
	if pct > 100 {
		return 100
	}
	return pct
}

func jointsClose(a, b [6]float64, tol float64) bool {
	for i := 0; i < 6; i++ {
		d := a[i] - b[i]
		if d < 0 {
			d = -d
		}
		if d > tol {
			return false
		}
	}
	return true
}

func orDefault[T int](v, def T) T {
	if v == 0 {
		return def
	}
	return v
}

// cr10a.urdf has 7 collision meshes: base_link + Link1..Link6. The RDK URDF
// parser assigns mesh_decimation_ratios per collision mesh in document order
// (base_link is index 0), so we need one ratio per mesh, not one per joint —
// a 6-element default would leave the heaviest wrist mesh (Link6) undecimated.
const (
	numCR10ACollisionMeshes = 7
	defaultMeshDecimation   = 0.1
)

// makeModelFrame builds the kinematic model from either the embedded capsule
// JSON (default) or the bundled URDF + meshes (when conf.UseURDF). The URDF
// path is resolved against VIAM_MODULE_ROOT, which viam-server sets to the
// unpacked module directory.
func makeModelFrame(conf *Config, name string) (referenceframe.Model, error) {
	if !conf.UseURDF {
		return referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, name)
	}
	ratios := conf.MeshDecimationRatios
	if len(ratios) == 0 {
		ratios = make([]float64, numCR10ACollisionMeshes)
		for i := range ratios {
			ratios[i] = defaultMeshDecimation
		}
	}
	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		return nil, errors.New("use_urdf is set but VIAM_MODULE_ROOT is empty")
	}
	path := filepath.Join(root, "arm", "cr10a.urdf")
	model, err := referenceframe.ParseModelXMLFile(path, name, ratios)
	if err != nil {
		return nil, fmt.Errorf("parsing URDF %q: %w", path, err)
	}
	return model, nil
}
