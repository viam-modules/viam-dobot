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
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(arm.API, Model, resource.Registration[arm.Arm, *Config]{
		Constructor: newCR10A,
	})
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

	speedFactor int
	jointSpeed  int
	jointAccel  int
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

	model, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, conf.ResourceName().ShortName())
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

func (a *cr10a) Get3DModels(_ context.Context, _ map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	// We don't ship STL meshes; collision geometry is the cylinders in the
	// kinematics JSON. Returning an empty map is the convention for arm
	// modules that have no 3D mesh assets.
	return map[string]*commonpb.Mesh{}, nil
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

// moveJoint is the single place that issues a JointMovJ. Callers may supply
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
	default:
		return nil, fmt.Errorf("unknown action %q", action)
	}
}

// ---------- helpers ----------

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
// The caller MUST hold a.mu.RLock — moveJoint takes it across the JointMovJ
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
