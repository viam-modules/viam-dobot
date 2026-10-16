// Manual (drag/freedrive) mode support for the live CR10A arm, built on the
// dashboard StartDrag()/StopDrag() commands. The dashboard protocol has no
// drag-state query, so the driver tracks the mode in a.manualMode; the tracked
// bool is kept in sync by SetManualMode, Stop, and the DoCommand drag actions.
package arm

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/grpc"
)

// exitTimer schedules a one-shot automatic exit from manual mode.
type exitTimer struct {
	mu sync.Mutex
	t  *time.Timer
}

func (e *exitTimer) schedule(d time.Duration, exit func()) {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.t != nil {
		e.t.Stop()
	}
	e.t = time.AfterFunc(d, exit)
}

func (e *exitTimer) cancel() {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.t != nil {
		e.t.Stop()
		e.t = nil
	}
}

// Properties reports which optional features the arm supports. Cartesian
// commands are supported via the armplanning.MoveArm path in MoveToPosition.
func (a *cr10a) Properties(_ context.Context, _ map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{SupportManualMode: true, SupportCartesianCommands: true}, nil
}

// ManualMode reports the tracked drag-mode state.
func (a *cr10a) ManualMode(_ context.Context, _ map[string]interface{}) (bool, error) {
	return a.manualMode.Load(), nil
}

// SetManualMode enters or exits drag/freedrive mode. When enabledFor > 0, the
// arm automatically exits manual mode after that duration; zero means no limit.
func (a *cr10a) SetManualMode(ctx context.Context, on bool, enabledFor time.Duration, _ map[string]interface{}) error {
	a.manualExit.cancel()
	if !on {
		return a.exitManualMode(ctx)
	}
	if err := a.enterManualMode(ctx); err != nil {
		return err
	}
	if enabledFor > 0 {
		a.manualExit.schedule(enabledFor, a.autoExitManualMode)
	}
	return nil
}

func (a *cr10a) enterManualMode(ctx context.Context) error {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if a.dash == nil {
		return errors.New("CR10A dashboard client not initialized")
	}
	if err := a.dash.startDrag(ctx); err != nil {
		// The controller refuses StartDrag while an alarm is latched.
		return fmt.Errorf("entering manual (drag) mode — call DoCommand clear_error first if an alarm is latched: %w", err)
	}
	a.manualMode.Store(true)
	return nil
}

func (a *cr10a) exitManualMode(ctx context.Context) error {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if a.dash == nil {
		return errors.New("CR10A dashboard client not initialized")
	}
	if err := a.dash.stopDrag(ctx); err != nil {
		return fmt.Errorf("exiting manual (drag) mode: %w", err)
	}
	a.manualMode.Store(false)
	return nil
}

// autoExitManualMode runs from the exitTimer when an enabledFor deadline expires.
func (a *cr10a) autoExitManualMode() {
	if !a.manualMode.Load() {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	if err := a.exitManualMode(ctx); err != nil {
		a.logger.Errorf("failed to automatically exit manual mode: %v", err)
	}
}

// Properties reports which optional features the simulated arm supports. Manual
// mode has nothing to back-drive; Cartesian commands go through armplanning.
func (s *simulatedCR10A) Properties(_ context.Context, _ map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{SupportManualMode: false, SupportCartesianCommands: true}, nil
}

// SetManualMode is unsupported for the simulated arm.
func (s *simulatedCR10A) SetManualMode(context.Context, bool, time.Duration, map[string]interface{}) error {
	return grpc.UnimplementedError
}

// ManualMode is unsupported for the simulated arm.
func (s *simulatedCR10A) ManualMode(context.Context, map[string]interface{}) (bool, error) {
	return false, grpc.UnimplementedError
}
