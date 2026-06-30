# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```sh
make            # build bin/viam-cr10a
make module     # build bin/module.tar.gz for `viam module upload`
make test       # go test ./...
make lint       # go vet ./... && gofmt -l . check
make clean

go test ./arm -run TestKinematicsParse   # single test
go test ./arm -run TestFeedback -v       # all feedback parsing tests
```

Build links against `libnlopt` (transitive dep of the RDK motion planner): `apt-get install libnlopt-dev` on Linux, `brew install nlopt` on macOS. Without it, `go build` will fail at the linker step, not at compile time.

The build targets in `meta.json` are `linux/amd64`, `linux/arm64`, `darwin/arm64`. The entrypoint is `bin/viam-cr10a`, and `make module` tars that binary together with `meta.json` at the archive root.

## Architecture

This is a Viam **arm component module**. `main.go` registers exactly one model — `viam-soleng:dobot:cr10a` — against `arm.API` and hands lifecycle to `module.ModularMain`. All real code lives in the `arm/` package.

### Three layers, three concerns

The `arm/` package is split into three files that correspond to the three concerns of the driver:

1. **`client.go` — dashboard client (TCP port 29999).** Synchronous ASCII request/response. Every command is `Cmd(arg1,arg2,...)\n` and every response is `ErrorID,{ResultList},CommandName;`. The `dashClient.mu` mutex serializes one command in flight at a time. Response framing uses a custom `bufio.Scanner` split function that breaks on `;`. All command wrappers use **degrees and millimeters** because that's what the wire protocol uses.

2. **`feedback.go` — feedback client (TCP port 30004).** Async, read-only. The controller broadcasts a packed 1440-byte little-endian struct at 125 Hz. We parse a small subset of fields (joint angles, TCP pose, robot_mode, enable/running/error status bytes) at known offsets. A magic value at offset 48 (`0x123456789ABCDEF`) is checked on every packet so torn frames are dropped silently. The reader runs in its own goroutine, reconnects with exponential backoff, and atomically publishes the latest valid frame via a `sync.RWMutex`.

3. **`cr10a.go` — `arm.Arm` implementation.** Wraps the two clients and consults a kinematic model loaded from `cr10a_kinematics.json` via `go:embed`. Converts radians ↔ degrees at the API boundary so callers only ever see SI units.

### Non-obvious design decisions

- **Unit conversion happens exactly at the `cr10a.go` ↔ `client.go` boundary.** `client.go` and `feedback.go` are pure wire-protocol code in degrees/mm; `cr10a.go` does `RadToDeg` on the way out and `DegToRad` on the way in. Don't add radian/SI handling inside the clients, and don't leak degrees out through `arm.Arm` methods.

- **Cartesian moves never reach the controller as `MovL` or pose-form `MovJ`.** `MoveToPosition` delegates to `armplanning.MoveArm`, which calls back into `Kinematics()` and ultimately `MoveToJointPositions` — which issues a joint-space `MovJ(joint={…})`. So the only `MovJ` the controller ever sees is the joint form; the kinematic model is the source of truth, not the device's reported TCP. `client.go` has no Cartesian (`MovL`/pose-`MovJ`) wrappers — adding a "send Cartesian directly" path means deciding to bypass the planner.

- **Motion completion is a two-phase poll.** `waitForMotionCompleteLocked` first waits up to `motionStartGrace` (200 ms) for `Running=true` to appear (so we don't race the controller starting up), then polls feedback at `motionPollInterval` (25 ms) until `Running=false` AND joints are within `jointToleranceDeg` (0.5°) of the target. There's also a one-poll re-check because the controller occasionally drops out of `Running` for a tick between waypoints. If you change the timing constants, preserve this two-phase shape. The `Locked` suffix means the caller (`moveJoint`) holds `a.mu.RLock` for the duration so `a.dash` is stable.

- **Cancellation propagates as `Stop()` on the wire.** When ctx is cancelled mid-move, the driver issues `Stop()` on a fresh 2-second context before returning `ctx.Err()`. `Stop()` is the V4 reference command for "halt the delivered motion command queue" — do **not** swap in `ResetRobot()` here (it resets the entire robot state) or `StopScript()` (a Magician/M1-series term, not in the V4 CR API). When the user explicitly calls the public `Stop()`, both that path and the in-flight `waitForMotionCompleteLocked` may issue `Stop()` on the wire — the second call may surface a controller error but the arm is correctly halted.

- **`MoveOptions` is honored** by mapping `MaxVelRads`/`MaxAccRads` to the controller's `VelJ`/`AccJ` 1..100 percent before each `MovJ(joint={…})`. The conversion uses `maxJointSpeedDegPerSec` and `maxJointAccelDegPerSec2` (both 180 °/s, the conservative across-joint floor for the CR10A); raise them if the controller is tuned for higher max speed and the planner consistently asks for slower moves than expected.

- **Latched alarms surface as errors, not as silent stalls.** The `error_status` byte in the feedback frame becomes `feedbackFrame.HasError`; `waitForMotionComplete` returns immediately with a "call DoCommand clear_error" message if it sees that flag set. The dashboard `expectOK` helper similarly turns any non-zero `ErrorID` into a Go error.

- **`Reconfigure` tears down and replaces both TCP clients.** It is the single source of truth for connection state — the constructor just calls it. After replacing clients, it does best-effort startup (`ClearError`, `SpeedFactor`, `VelJ`, `AccJ`, optionally `EnableRobot`) and waits up to 3 s for the first feedback frame so subsequent calls don't race.

- **Kinematics has two sources.** The default is the embedded capsule JSON (`cr10a_kinematics.json`, `//go:embed`-baked into the binary; `Reconfigure` calls `UnmarshalModelJSON` on every call). When `use_urdf: true` is set, `makeModelFrame` instead calls `referenceframe.ParseModelXMLFile` on `$VIAM_MODULE_ROOT/arm/cr10a.urdf`, passing `mesh_decimation_ratios` (7 ratios, one per collision mesh in document order — `base_link` then `Link1`–`Link6` — defaulting to 0.1 each). `make module` bundles `arm/cr10a.urdf` + `arm/meshes/` into the tarball so the meshes are available on the target host. `TestJSONURDFForwardKinematicsAgree` guards that both models produce the same tool pose (verified to agree within ~4 µm / ~5×10⁻⁴ °). The URDF's `dummy_joint` carries an explicit zero `<origin>` as a workaround for an RDK v0.123.0 URDF-parser nil-deref; do not remove it on re-import. The embedded JSON path remains the default until the URDF is confirmed on hardware.

- **`Get3DModels` always serves the link meshes.** It reads the 7 bundled STLs from `$VIAM_MODULE_ROOT/arm/meshes/cr10/`, converts each to PLY (`spatialmath.NewMeshFromSTLFile` → `TrianglesToPLYBytes(false)`, `ContentType: "ply"`), and keys them by the active model's frame names — the UR-style JSON names (`shoulder_link`, …) when `use_urdf` is false, the URDF names (`Link1`…`Link6`) when true. This works in both modes because the JSON and URDF link frames coincide (`TestPerLinkFrameAlignment` guards it). `cr10aMeshParts` is the single source of truth for the per-link `(stlFile, jsonName, urdfName)` triple; the result is cached on the `cr10a` struct keyed by the active mode and invalidated by `Reconfigure`. With `VIAM_MODULE_ROOT` unset it warns and returns an empty map rather than failing.

### DoCommand is the escape hatch

The `arm.Arm` interface doesn't expose Dobot-specific actions, so `enable`/`disable`/`clear_error`/`emergency_stop`/`set_speed`/`robot_mode`/`start_drag`/`stop_drag`/`set_drag_sensitivity` go through `DoCommand`. The action is a string under the `"action"` key. JSON numbers come in as `float64` — note the cast in the `set_speed` and `set_drag_sensitivity` handlers. When adding new actions, follow the same pattern (string action key, return `{"ok": true}` on success, surface errors directly). The drag actions map to the `StartDrag()`/`StopDrag()`/`DragSensivity(index,value)` wire commands — the controller refuses drag while an alarm is latched, so `clear_error` first.

## Testing notes

The unit tests are scoped to logic that doesn't need the controller:

- `TestKinematicsParse` — embedded JSON parses, has 6 DoF, zero pose is finite and within reach.
- `TestFeedbackParse` / `TestFeedbackRejectsBadMagic` — synthetic 1440-byte packet, exercises offset math and the magic check.
- `TestParseDashResp` — response parser handles the various reply shapes.

There are no integration tests against a real CR10A; bring-up is manual (see README "Hardware bring-up checklist"). When adding new wire-protocol command wrappers in `client.go`, add a `parseDashResp` test for any response shape the existing cases don't already cover.
