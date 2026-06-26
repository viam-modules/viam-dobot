> **Superseded (point-in-time review).** This document is a historical snapshot from an
> earlier review and describes code that has since changed. In particular, the V4 wire
> commands were corrected to `VelJ` / `MovJ(joint={…})` (not `SpeedJ` / `JointMovJ`) and the
> feedback status-byte offsets to `1026/1028/1029` on the `fix/v4-wire-protocol` branch.
> Treat the specifics below as dated; the current code and `CLAUDE.md` are authoritative.

## Verdict

**It's a legitimate Viam arm module.** Builds clean (`go build ./...` and `make test` pass), registers correctly via `module.ModularMain`, implements the full `arm.Arm` surface, has the right shape vs. both UR and xArm. But it was written without ever talking to a real controller, and there are a few real bugs that unit tests won't catch.

## What's done right

| Pattern | dobot | xArm reference | UR reference |
|---|---|---|---|
| Module entrypoint | `module.ModularMain` ✓ | same | n/a (C++) |
| `arm.API` registration with proper `ModelNamespace().WithFamily().WithModel()` | ✓ | ✓ | ✓ |
| `Config.Validate` | ✓ host required, percent ranges checked | ✓ | ✓ |
| `operation.SingleOperationManager` | ✓ used in `MoveToPosition`, `MoveToJointPositions` | ✓ | analog |
| Kinematics via `go:embed` | ✓ `cr10a.go:40-41` | ✓ | from disk |
| Unit boundary discipline | ✓ deg/mm in `client.go`+`feedback.go`, rad in `cr10a.go` | analog | ✓ |
| Two-layer wire (synchronous dashboard + async realtime feedback) | ✓ | n/a (xArm is poll-only) | ✓ same shape |
| `MoveToPosition` → `armplanning.MoveArm` | ✓ — and this is **better** than xArm's pattern, which forces injecting `motion.Service` as a dep | uses `motion.Move` (older) | uses URCL |
| Cancel ctx → wire stop | ✓ `cr10a.go:451-460` | not really — `xarm/comm.go:831` returns `ctx.Err()` without stopping | ✓ TRAJECTORY_CANCEL |
| Latched-alarm surfacing via `error_status` byte | ✓ | ✓ via state register | ✓ via safety bits |
| `Reconfigure` replaces both TCP clients on host change | ✓ | xArm uses `AlwaysRebuild` — different but valid | ✓ |

The protocol parsing matches the published Dobot CR-series spec (29999 ASCII command/ack, 30004 1440-byte packed binary at 125 Hz, magic at offset 48, joint angles at 432, status bytes at 1026/1028/1029). The kinematics JSON is structurally well-formed and link lengths match the README's URDF-derived numbers.

## Bugs you should care about

**1. Feedback "have" flag never resets on disconnect.** `feedback.go:164` is the only writer, and it only sets `true`. If the 30004 socket drops mid-motion (controller reboot, network blip, anything), `feedback.latest()` keeps returning the last frame as if it's live. Consequences:
- `waitForMotionComplete` (`cr10a.go:430-487`) sees stale `Running=true` and spins until ctx cancellation
- `IsMoving` (`cr10a.go:358-364`) lies about motion state
- `JointPositions` returns stale joint angles forever

Fix: in `recordError()` (`feedback.go:173-177`) or when `readLoop` returns, also set `f.have = false` (or carry a "frame age" deadline that callers check).

**2. `MoveOptions` is dropped on the floor.** `MoveThroughJointPositions` (`cr10a.go:322-337`) ignores `*arm.MoveOptions`, so caller-supplied `MaxVelRads` / `MaxAccRads` never reach the controller. The motion planner relies on these for velocity profiling. xArm honors them at `xarm.go:500-508`. You'd need to map them to `SpeedJ`/`AccJ` calls before issuing `JointMovJ`.

**3. `StopScript()` is probably not the right command.** The Dobot CR-series TCP/IP protocol calls it `StopRobot()` (or `MoveStop`); `StopScript()` is a Magician/M1-series term. The fallback to `ResetRobot()` (`client.go:226-232`) is heavy: `ResetRobot` resets the entire robot state, not just halts motion. This needs to be verified against your CR10A's firmware before you trust ctx-cancel or `Stop()` in production. The author guessed; it might happen to work, but verify.

**4. `dashClient` send/close race.** `Close()` holds `a.mu` exclusive (`cr10a.go:217-225`), but motion methods do `a.mu.RLock(); dash := a.dash; a.mu.RUnlock(); dash.X(...)` — the dash call happens *outside* the lock. So `Close()` can run between the unlock and the dash call, closing the conn while a write is mid-flight. Result is just a write error (not a crash), but it's avoidable. Hold the read lock through the wire call, or reference-count the dash client.

**5. `ServoJ` uses keyword args** (`client.go:271-275`: `t=%.4f,lookahead_time=%.0f,gain=%.0f`). The wire protocol is positional. Currently dead code (nothing calls `servoJ`), but if anyone ever wires it up it'll fail. Either delete or fix.

**6. `stopScript()` runs twice on cancel.** `Stop()` calls `opMgr.CancelRunning()` then `dash.stopScript()`, and the in-flight `waitForMotionComplete` *also* calls `dash.stopScript()` when it sees ctx.Done (`cr10a.go:455-459`). Harmless, but the second call may error against the now-stopped controller.

**7. `set_speed` DoCommand updates the wire but not `a.speedFactor`.** A subsequent `Reconfigure` reverts it. Inconsistent with how speed is treated as persistent state.

**8. `IsMoving` and `latestFrame` swallow feedback errors** (the `_` on the third return of `latest()`). Combined with bug #1, you can't tell "running" from "disconnected and last frame said running."

## Things that aren't bugs but warrant a hardware check before you trust them

- **Kinematics correctness.** `TestKinematicsParse` only asserts the home pose is finite and within 5 m of the base. It doesn't pin specific link transforms. The quaternion conventions on `shoulder_link`/`forearm_link`/`wrist_*_link` are the kind of thing a one-shot can plausibly get wrong (axis flips, rotation handedness). Before trusting Cartesian moves, sanity-check by setting `JointMovJ(0,0,0,0,0,0)` on the real arm and comparing reported TCP against `EndPosition()`.
- **Joint limits are conservative.** README acknowledges this. Widen in the JSON if your application needs it.
- **`Get3DModels` returns empty.** Author chose this intentionally (cylinders in kinematics suffice for collision); UR ships GLB files. Acceptable.
- **No drag-teach/freedrive.** Author flagged it. Easy DoCommand addition if needed.
- **`motionStartGrace` of 200ms.** Some Dobot firmware versions take longer to dispatch. If you see `waitForMotionComplete` short-circuiting through the `goto completed` on real hardware, bump it.

## What I'd do next

In order of importance:

1. **Bring it up against a real CR10A** and exercise `MoveToJointPositions(0,0,0,0,0,0)` followed by `EndPosition()` — verify the reported pose matches the URDF prediction. Repeat for a non-trivial joint config. This catches both the kinematics bugs and bug #3 (verify ctx-cancel actually halts motion).
2. **Fix bug #1** — it will bite you the first time the controller reboots while a move is queued.
3. **Honor `MoveOptions`** (#2) so the motion planner can control velocity.
4. **Verify `StopRobot()` vs `StopScript()`** (#3) against the real firmware and clean up the fallback.

The CLAUDE.md I wrote earlier is accurate to what the code does, but its "Cancellation propagates as `StopScript()`" claim inherits the unverified assumption from bug #3 — worth softening to "issues a stop command (currently `StopScript()`, falling back to `ResetRobot()`); verify against your firmware" once you confirm.
