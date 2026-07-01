package arm

import (
	"context"
	"math"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

// newTestSimArm constructs a simulated CR10A with the simulated clock disabled, so
// tests drive time deterministically via updateForTime. speedRadPerSec sets the
// joint speed in radians per second (1.0 makes interpolation arithmetic exact).
func newTestSimArm(t *testing.T, speedRadPerSec float64) *simulatedCR10A {
	t.Helper()
	simulateTime := false
	conf := resource.Config{
		Name:  "testSimArm",
		API:   arm.API,
		Model: SimulatedModel,
		ConvertedAttributes: &SimulatedConfig{
			SpeedDegsPerSec: speedRadPerSec * 180.0 / math.Pi,
			SimulateTime:    &simulateTime,
		},
	}
	a, err := newSimulatedCR10A(context.Background(), nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatalf("newSimulatedCR10A: %v", err)
	}
	return a.(*simulatedCR10A)
}

func waitForMoving(t *testing.T, a arm.Arm) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		moving, err := a.IsMoving(context.Background())
		if err != nil {
			t.Fatalf("IsMoving: %v", err)
		}
		if moving {
			return
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatal("arm never started moving")
}

func TestSimulatedConfigValidate(t *testing.T) {
	t.Run("default config declares no dependencies", func(t *testing.T) {
		deps, optional, err := (&SimulatedConfig{}).Validate("")
		if err != nil {
			t.Fatalf("Validate: %v", err)
		}
		if len(deps) != 0 || len(optional) != 0 {
			t.Fatalf("expected no deps, got required=%v optional=%v", deps, optional)
		}
	})

	t.Run("negative speed is rejected", func(t *testing.T) {
		if _, _, err := (&SimulatedConfig{SpeedDegsPerSec: -1}).Validate(""); err == nil {
			t.Fatal("expected error for negative speed")
		}
	})

	t.Run("out-of-range mesh decimation ratio is rejected", func(t *testing.T) {
		if _, _, err := (&SimulatedConfig{MeshDecimationRatios: []float64{1.5}}).Validate(""); err == nil {
			t.Fatal("expected error for ratio > 1")
		}
	})
}

func TestSimulatedKinematics(t *testing.T) {
	sim := newTestSimArm(t, 1.0)
	defer func() {
		if err := sim.Close(context.Background()); err != nil {
			t.Fatalf("Close: %v", err)
		}
	}()

	model, err := sim.Kinematics(context.Background())
	if err != nil {
		t.Fatalf("Kinematics: %v", err)
	}
	// The CR10A arm has 6 revolute joints.
	if got := len(model.DoF()); got != 6 {
		t.Fatalf("expected 6 DoF, got %d", got)
	}

	inputs, err := sim.CurrentInputs(context.Background())
	if err != nil {
		t.Fatalf("CurrentInputs: %v", err)
	}
	if len(inputs) != 6 {
		t.Fatalf("expected 6 inputs, got %d", len(inputs))
	}
	for i, in := range inputs {
		if in != 0 {
			t.Errorf("input %d: expected 0, got %v", i, in)
		}
	}
}

func TestSimulatedMoveToJointPositions(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0) // 1 radian/second
	defer func() {
		if err := sim.Close(ctx); err != nil {
			t.Fatalf("Close: %v", err)
		}
	}()

	// Joint 1 must travel 1.0 rad (the farthest), so the move takes 1 second. Joint 0
	// travels half as far, so it moves at half speed.
	target := []referenceframe.Input{0.5, -1.0, 0, 0, 0, 0}

	moveErr := make(chan error, 1)
	go func() { moveErr <- sim.MoveToJointPositions(ctx, target, nil) }()
	waitForMoving(t, sim)

	// Advance the simulated clock half a second: the move should be half complete.
	base := time.Time{}
	sim.updateForTime(base.Add(500 * time.Millisecond))
	inputs, err := sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatalf("JointPositions: %v", err)
	}
	assertInDelta(t, inputs, []float64{0.25, -0.5, 0, 0, 0, 0}, 1e-9)

	select {
	case <-moveErr:
		t.Fatal("MoveToJointPositions returned before the move completed")
	default:
	}

	// Advance to one second: the move should be complete and the call should return.
	sim.updateForTime(base.Add(time.Second))
	inputs, err = sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatalf("JointPositions: %v", err)
	}
	assertInDelta(t, inputs, []float64{0.5, -1.0, 0, 0, 0, 0}, 1e-9)

	select {
	case err := <-moveErr:
		if err != nil {
			t.Fatalf("MoveToJointPositions returned error: %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("MoveToJointPositions did not return after the move completed")
	}

	moving, err := sim.IsMoving(ctx)
	if err != nil {
		t.Fatalf("IsMoving: %v", err)
	}
	if moving {
		t.Fatal("expected arm to be stopped after move completed")
	}
}

func TestSimulatedMoveRejectsWrongJointCount(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { _ = sim.Close(ctx) }()

	if err := sim.MoveToJointPositions(ctx, []referenceframe.Input{0, 0, 0}, nil); err == nil {
		t.Fatal("expected error for wrong joint count")
	}
}

func TestSimulatedStop(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { _ = sim.Close(ctx) }()

	moveErr := make(chan error, 1)
	go func() {
		moveErr <- sim.MoveToJointPositions(ctx, []referenceframe.Input{0.5, -1.0, 0, 0, 0, 0}, nil)
	}()
	waitForMoving(t, sim)

	if err := sim.Stop(ctx, nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}

	select {
	case err := <-moveErr:
		if err == nil || !strings.Contains(err.Error(), "stopped before reaching target") {
			t.Fatalf("expected stop error, got %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("MoveToJointPositions did not return after Stop")
	}
}

func TestSimulatedEndPosition(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { _ = sim.Close(ctx) }()

	pose, err := sim.EndPosition(ctx, nil)
	if err != nil {
		t.Fatalf("EndPosition: %v", err)
	}
	if pose == nil {
		t.Fatal("expected non-nil pose")
	}

	// Moving a joint should change the end-effector pose.
	sim.mu.Lock()
	sim.currInputs = []float64{1.0, 0, 0, 0, 0, 0}
	sim.mu.Unlock()

	moved, err := sim.EndPosition(ctx, nil)
	if err != nil {
		t.Fatalf("EndPosition: %v", err)
	}
	if pose.Point().X == moved.Point().X &&
		pose.Point().Y == moved.Point().Y &&
		pose.Point().Z == moved.Point().Z {
		t.Fatal("end position should change after a joint moves")
	}
}

func TestSimulatedGet3DModels(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "..")
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { _ = sim.Close(ctx) }()

	models, err := sim.Get3DModels(ctx, nil)
	if err != nil {
		t.Fatalf("Get3DModels: %v", err)
	}
	// One GLB per link mesh: base_link + Link1..Link6.
	if len(models) != len(cr10aMeshParts) {
		t.Fatalf("expected %d meshes, got %d", len(cr10aMeshParts), len(models))
	}
	for _, part := range []string{"base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "ee_link"} {
		mesh, ok := models[part]
		if !ok {
			t.Errorf("missing mesh for frame %q", part)
			continue
		}
		if mesh.ContentType != "model/gltf-binary" {
			t.Errorf("frame %q: expected content type model/gltf-binary, got %q", part, mesh.ContentType)
		}
		if len(mesh.Mesh) == 0 || string(mesh.Mesh[:4]) != "glTF" {
			t.Errorf("frame %q: expected non-empty glTF payload", part)
		}
	}
}

func TestSimulatedTimeSimulation(t *testing.T) {
	ctx := context.Background()
	// With simulate_time left at its default (true), the background goroutine advances
	// the arm on its own, so MoveToJointPositions completes without manual updateForTime.
	conf := resource.Config{
		Name:  "testSimArm",
		API:   arm.API,
		Model: SimulatedModel,
		ConvertedAttributes: &SimulatedConfig{
			SpeedDegsPerSec: 2000, // fast, so the move finishes quickly
		},
	}
	a, err := newSimulatedCR10A(ctx, nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatalf("newSimulatedCR10A: %v", err)
	}
	defer func() { _ = a.Close(ctx) }()

	target := []referenceframe.Input{0.3, -0.3, 0.2, 0, 0, 0}
	if err := a.MoveToJointPositions(ctx, target, nil); err != nil {
		t.Fatalf("MoveToJointPositions: %v", err)
	}

	inputs, err := a.JointPositions(ctx, nil)
	if err != nil {
		t.Fatalf("JointPositions: %v", err)
	}
	assertInDelta(t, inputs, []float64{0.3, -0.3, 0.2, 0, 0, 0}, 1e-6)

	moving, err := a.IsMoving(ctx)
	if err != nil {
		t.Fatalf("IsMoving: %v", err)
	}
	if moving {
		t.Fatal("expected arm to be stopped after move completed")
	}
}

// assertInDelta fails the test if any element of got differs from want by more
// than delta, or if the lengths differ.
func assertInDelta(t *testing.T, got []referenceframe.Input, want []float64, delta float64) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("length mismatch: got %d, want %d", len(got), len(want))
	}
	for i := range want {
		if math.Abs(float64(got[i])-want[i]) > delta {
			t.Errorf("index %d: got %v, want %v (±%v)", i, got[i], want[i], delta)
		}
	}
}
