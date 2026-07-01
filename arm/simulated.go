// This file implements a hardware-free, simulated CR10A arm. It shares the
// kinematic model and link meshes with the live viam:dobot:cr10a model but
// needs no controller: joint motion is interpolated in software against a
// realtime clock. It is meant for testing configs, motion plans, and the 3D
// scene viewer while away from a physical arm.
//
// The design mirrors the SO-101 module's simulated arm (../so-101/simulated.go):
// a background goroutine advances joint positions toward the target at a fixed
// top speed, and MoveToJointPositions blocks until the move completes, is
// stopped, or the context is canceled. MoveToPosition delegates to the same
// armplanning.MoveArm path the live arm uses, so Cartesian goals are planned in
// joint space against the shared kinematic model.
package arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"os"
	"sync"
	"sync/atomic"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// SimulatedModel is the model triplet for the hardware-free simulated CR10A arm.
// The wire model string is `<namespace>:dobot:cr10a-simulated`.
var SimulatedModel = resource.ModelNamespace("viam").WithFamily("dobot").WithModel("cr10a-simulated")

// defaultSimSpeedDegsPerSec is the joint travel speed used when speed_degs_per_sec
// is unset. 60°/s is a deliberately gentle default for visualization.
const defaultSimSpeedDegsPerSec = 60.0

// simTimeInterval is how often the background goroutine advances the arm's position.
const simTimeInterval = 10 * time.Millisecond

func init() {
	resource.RegisterComponent(arm.API, SimulatedModel,
		resource.Registration[arm.Arm, *SimulatedConfig]{
			Constructor: newSimulatedCR10A,
		},
	)
}

// SimulatedConfig configures a simulated CR10A arm. It needs no hardware, so it
// shares only the kinematics-related options with the live Config — there is no
// host/port. UseURDF and MeshDecimationRatios select the kinematic source and
// mesh decimation exactly as on the live model, so the simulated arm renders and
// plans against the same geometry.
type SimulatedConfig struct {
	// SpeedDegsPerSec is how fast each joint travels toward its target, in degrees
	// per second. Defaults to defaultSimSpeedDegsPerSec when unset.
	SpeedDegsPerSec float64 `json:"speed_degs_per_sec,omitempty"`

	// SimulateTime controls whether a background goroutine advances the arm's
	// position in real time. Defaults to true. Tests set it false to drive the
	// simulated clock deterministically via updateForTime.
	SimulateTime *bool `json:"simulate_time,omitempty"`

	// UseURDF loads kinematics + meshes from arm/cr10a.urdf instead of the
	// embedded capsule JSON, mirroring the live model's option. Requires
	// VIAM_MODULE_ROOT to be set.
	UseURDF bool `json:"use_urdf,omitempty"`

	// MeshDecimationRatios is the per-collision-mesh simplification ratio in
	// [0,1] used when UseURDF is set. See Config.MeshDecimationRatios for the
	// full semantics. Ignored unless UseURDF is true.
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`
}

// Validate satisfies resource.ConfigValidator. The simulated arm has no
// dependencies.
func (cfg *SimulatedConfig) Validate(path string) ([]string, []string, error) {
	if cfg.SpeedDegsPerSec < 0 {
		return nil, nil, fmt.Errorf("speed_degs_per_sec must not be negative, got %.1f", cfg.SpeedDegsPerSec)
	}
	for i, r := range cfg.MeshDecimationRatios {
		if math.IsNaN(r) || r < 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratios[%d] must be in [0, 1], got %f", i, r)
		}
	}
	return nil, nil, nil
}

// simOperation tracks the state of an in-flight MoveToJointPositions request.
//
// Logical states/invariants:
//  1. Default constructed -- no operation in flight.
//  2. Operation started -> targetInputs != nil, done == false, stopped == false.
//  3. Operation successful -> done == true.
//  4. Operation stopped -> stopped == true.
type simOperation struct {
	// targetInputs is the goal joint configuration in radians.
	targetInputs []float64
	done         bool
	stopped      bool
}

func (op simOperation) isMoving() bool {
	return op.targetInputs != nil && !op.done && !op.stopped
}

// simulatedCR10A is a hardware-free CR10A arm. It shares the kinematic model and
// link meshes with the live cr10a model and interpolates joint motion over time,
// mirroring the behavior of rdk's builtin "simulated" arm.
type simulatedCR10A struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	model  referenceframe.Model

	// speed is the joint travel speed in radians per second.
	speed float64

	// meshUseURDF selects which frame-name set Get3DModels keys the bundled
	// meshes by: the URDF names when true, the JSON (UR-derived) names when false.
	meshUseURDF bool

	// lifetime management
	closed     atomic.Bool
	cancelCtx  context.Context
	cancelFunc func()
	workers    sync.WaitGroup

	// mu guards the fields below.
	mu          sync.Mutex
	currInputs  []float64 // current joint positions in radians, length == model DoF
	lastUpdated time.Time
	operation   simOperation
}

func newSimulatedCR10A(
	ctx context.Context, _ resource.Dependencies, rawConf resource.Config, logger logging.Logger,
) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*SimulatedConfig](rawConf)
	if err != nil {
		return nil, err
	}

	// Reuse the live model's kinematics builder; it only reads UseURDF and
	// MeshDecimationRatios off the Config.
	model, err := makeModelFrame(&Config{
		UseURDF:              conf.UseURDF,
		MeshDecimationRatios: conf.MeshDecimationRatios,
	}, rawConf.ResourceName().ShortName())
	if err != nil {
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	speedDegsPerSec := conf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = defaultSimSpeedDegsPerSec
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	sim := &simulatedCR10A{
		Named:       rawConf.ResourceName().AsNamed(),
		logger:      logger,
		model:       model,
		speed:       speedDegsPerSec * math.Pi / 180.0,
		meshUseURDF: conf.UseURDF,
		cancelCtx:   cancelCtx,
		cancelFunc:  cancelFunc,
		currInputs:  make([]float64, len(model.DoF())),
	}

	// SimulateTime defaults to true so a deployed arm advances on its own.
	if conf.SimulateTime == nil || *conf.SimulateTime {
		// Avoid ever letting the zero value of lastUpdated be visible, lest the
		// first movement be unpredictable.
		sim.lastUpdated = time.Now()
		sim.startTimeSimulation()
	}

	logger.Debugf("simulated CR10A configured with speed: %.1f deg/s (use_urdf=%t)", speedDegsPerSec, conf.UseURDF)
	return sim, nil
}

// startTimeSimulation launches a background goroutine that advances the arm's
// position against a realtime clock until the arm is closed.
func (s *simulatedCR10A) startTimeSimulation() {
	s.workers.Add(1)
	go func() {
		defer s.workers.Done()
		ticker := time.NewTicker(simTimeInterval)
		defer ticker.Stop()
		for {
			select {
			case <-s.cancelCtx.Done():
				return
			case <-ticker.C:
				s.updateForTime(time.Now())
			}
		}
	}()
}

// updateForTime advances the simulated joint positions to the given wall-clock
// time. It is called by the background goroutine when simulate_time is true, and
// directly by tests for a deterministic clock when it is false.
func (s *simulatedCR10A) updateForTime(now time.Time) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.operation.isMoving() {
		s.lastUpdated = now
		return
	}

	timeSinceLastUpdate := now.Sub(s.lastUpdated)
	s.lastUpdated = now

	// Find the maximum joint travel distance. Because all joints move at the same
	// top speed, this maps to how long the whole movement takes.
	var maxDist float64
	for jointIdx, currJointInp := range s.currInputs {
		maxDist = math.Max(maxDist, math.Abs(s.operation.targetInputs[jointIdx]-currJointInp))
	}

	const epsilon = 1e-9
	if maxDist < epsilon {
		s.operation.done = true
		return
	}

	// Scale each joint's speed so that every joint finishes its travel at the same
	// time. This matches rdk's motion-planning interpolation.
	modifiedSpeeds := make([]float64, len(s.currInputs))
	for jointIdx, currJointInp := range s.currInputs {
		diffRads := math.Abs(s.operation.targetInputs[jointIdx] - currJointInp)
		modifiedSpeeds[jointIdx] = (diffRads / maxDist) * s.speed
	}

	// anyJointStillMoving stays false only when every joint has reached its target.
	anyJointStillMoving := false
	for jointIdx, currJointInp := range s.currInputs {
		// Signed remaining distance to the target.
		diffRads := s.operation.targetInputs[jointIdx] - currJointInp

		// How far this joint could travel since the last update, capped at diffRads.
		toTravelRads := timeSinceLastUpdate.Seconds() * modifiedSpeeds[jointIdx]
		if toTravelRads > math.Abs(diffRads)-epsilon {
			// We can travel at least as far as we need to; snap to the target.
			s.currInputs[jointIdx] = s.operation.targetInputs[jointIdx]
		} else {
			if diffRads < 0 {
				// toTravelRads is always positive; flip it to travel the other way.
				toTravelRads = -toTravelRads
			}
			s.currInputs[jointIdx] = currJointInp + toTravelRads
			anyJointStillMoving = true
		}
	}

	if !anyJointStillMoving {
		s.operation.done = true
	}
}

// EndPosition returns the pose of the end effector at the current joint positions.
func (s *simulatedCR10A) EndPosition(ctx context.Context, _ map[string]interface{}) (spatialmath.Pose, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return s.model.Transform(inputs)
}

// MoveToPosition moves the end effector to the target pose. It delegates to the
// same planner the live arm uses: armplanning.MoveArm computes a joint trajectory
// from Kinematics() and drives it via MoveToJointPositions.
func (s *simulatedCR10A) MoveToPosition(ctx context.Context, pose spatialmath.Pose, _ map[string]interface{}) error {
	return armplanning.MoveArm(ctx, s.logger, s, pose)
}

// MoveToJointPositions starts a move to the given joint configuration and blocks
// until it completes, the arm is stopped, or the context is canceled.
func (s *simulatedCR10A) MoveToJointPositions(
	ctx context.Context, positions []referenceframe.Input, _ map[string]interface{},
) error {
	if len(positions) != len(s.model.DoF()) {
		return fmt.Errorf("expected %d joint positions for the CR10A arm, got %d",
			len(s.model.DoF()), len(positions))
	}
	if err := arm.CheckDesiredJointPositions(ctx, s, positions); err != nil {
		return err
	}

	target := make([]float64, len(positions))
	copy(target, positions)

	s.mu.Lock()
	s.operation = simOperation{targetInputs: target}
	s.mu.Unlock()

	// MoveToJointPositions blocks until the movement completes or is canceled.
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-s.cancelCtx.Done():
			return s.cancelCtx.Err()
		default:
			s.mu.Lock()
			done, stopped := s.operation.done, s.operation.stopped
			s.mu.Unlock()

			if done {
				return nil
			}
			if stopped {
				return errors.New("stopped before reaching target")
			}
			time.Sleep(time.Millisecond)
		}
	}
}

// MoveThroughJointPositions moves the arm through each joint configuration in order.
func (s *simulatedCR10A) MoveThroughJointPositions(
	ctx context.Context, positions [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{},
) error {
	for i, goal := range positions {
		if err := s.MoveToJointPositions(ctx, goal, nil); err != nil {
			return fmt.Errorf("waypoint %d: %w", i, err)
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
	}
	return nil
}

// JointPositions returns the current joint positions in radians.
func (s *simulatedCR10A) JointPositions(_ context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	ret := make([]referenceframe.Input, len(s.currInputs))
	copy(ret, s.currInputs)
	return ret, nil
}

func (s *simulatedCR10A) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *simulatedCR10A) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *simulatedCR10A) Kinematics(_ context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

// Stop ends any in-flight movement. The arm holds its current position.
func (s *simulatedCR10A) Stop(_ context.Context, _ map[string]interface{}) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Only set stopped while moving, otherwise the distinction between "reached the
	// goal" and "was stopped" is lost.
	if s.operation.isMoving() {
		s.operation.stopped = true
	}
	return nil
}

func (s *simulatedCR10A) IsMoving(_ context.Context) (bool, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	return s.operation.isMoving(), nil
}

// Geometries returns the arm's geometries at the current joint positions.
func (s *simulatedCR10A) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

// Get3DModels returns the bundled CR10A link meshes as GLB for the 3D scene
// viewer, keyed by the active model's frame names — identical output to the live
// model's Get3DModels. With VIAM_MODULE_ROOT unset it warns and returns an empty
// map rather than failing.
func (s *simulatedCR10A) Get3DModels(_ context.Context, _ map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		s.logger.Warn("Get3DModels: VIAM_MODULE_ROOT is not set; returning empty mesh map")
		return map[string]*commonpb.Mesh{}, nil
	}
	return buildMeshMap(root, s.meshUseURDF)
}

func (s *simulatedCR10A) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "get_motion_params":
		return map[string]interface{}{
			"speed_degs_per_sec": s.speed * 180.0 / math.Pi,
		}, nil
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (s *simulatedCR10A) Close(_ context.Context) error {
	if s.closed.Swap(true) {
		return nil
	}
	s.cancelFunc()
	s.workers.Wait()
	return nil
}
