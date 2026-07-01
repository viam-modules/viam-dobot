# viam-dobot-cr10a

A Viam **arm** module for the Dobot CR10A 6-axis collaborative robot. The
driver speaks the native CR-series TCP/IP remote-control protocol — no
DobotStudio or ROS bridge in the loop — and ships a Viam kinematics model
derived from Dobot's official URDF, so the motion service can plan paths,
check collisions, and inverse-kinematic against the real link geometry.

Registered models: `viam:dobot:cr10a` (the live controller driver) and
`viam:dobot:cr10a-simulated` (a hardware-free [simulated arm](#simulated-arm)
for testing motions without a connected robot).

## Features

- **Native TCP/IP protocol.** Dashboard port 29999 for commands, real-time
  port 30004 for the 1440-byte / 125 Hz feedback broadcast. Two sockets,
  no scripting layer.
- **Full Viam motion-planning integration.** Implements
  `framesystem.InputEnabled`, `Geometries`, and `Kinematics`, so the motion
  service plans in joint space using the supplied SVA model.
- **Safe blocking semantics.** `MoveToJointPositions` issues `MovJ(joint={...})`
  and blocks until the controller's `running_status` clears AND the encoder
  values are within tolerance of the target — so callers can chain moves
  without races.
- **Cancellation via context.** Cancel ctx during a move and the driver
  issues `Stop()` before returning `ctx.Err()`.
- **Latched-alarm detection.** If the controller raises an error mid-move,
  the call returns a descriptive error; reset with `DoCommand({"action":"clear_error"})`.

## Configure

Example attributes configuration:

```json
{
  "host": "192.168.1.6",
  "speed_factor": 30,
  "joint_speed":  50,
  "joint_accel":  50,
  "auto_enable":  true
}
```

| Attribute        | Type | Default | Description |
|------------------|------|--------:|-------------|
| `host`           | str  |  *(required)* | Controller IP address. |
| `dashboard_port` | int  | 29999 | Command/ack port. |
| `feedback_port`  | int  | 30004 | Real-time broadcast port. |
| `speed_factor`   | int  | 50    | Global `SpeedFactor()` percent (1..100). |
| `joint_speed`    | int  | 50    | Per-`MovJ` `VelJ` percent. |
| `joint_accel`    | int  | 50    | Per-`MovJ` `AccJ` percent. |
| `auto_enable`    | bool | true  | Issue `EnableRobot()` at module start. |
| `use_urdf`       | bool | false | When `true`, loads kinematics and visual/collision meshes from the bundled `arm/cr10a.urdf` instead of the default embedded capsule-geometry JSON. Gives a richer 3D visualization in the Viam app and mesh-accurate collision geometry for motion planning, at the cost of slower planning. Requires `VIAM_MODULE_ROOT` to be set (automatic when run via `viam-server`). Default remains `false` until the URDF is validated on hardware. |
| `mesh_decimation_ratios` | array of numbers | `[0.1, …]` (7 entries) | Per-collision-mesh simplification ratio in URDF document order (`base_link` followed by `Link1`–`Link6`). Only ratios strictly between 0 and 1 (exclusive) actually simplify a mesh — a value of `0` or `1` leaves that mesh at full resolution; within (0,1), smaller values simplify more aggressively. Supply one ratio per mesh (7 total); if you provide fewer than 7, trailing meshes are left at full resolution. Only used when `use_urdf` is `true`. Omit to accept the default 0.1 for all 7 meshes. |

### DoCommand actions

The arm.Arm interface doesn't expose Dobot-specific actions, so they're
available via `DoCommand`:

| Command JSON                                | Effect |
|--------------------------------------------|--------|
| `{"action": "enable"}`                      | `EnableRobot()` |
| `{"action": "disable"}`                     | `DisableRobot()` |
| `{"action": "clear_error"}`                 | `ClearError()` (use after a latched alarm) |
| `{"action": "emergency_stop"}`              | Software E-stop |
| `{"action": "set_speed", "value": 1..100}`  | `SpeedFactor(value)` |
| `{"action": "robot_mode"}`                  | Returns `{"mode": <int>}` per the CR-series RobotMode enum |
| `{"action": "start_drag"}`                  | `StartDrag()` — enter drag/freedrive (refused while an alarm is latched; `clear_error` first) |
| `{"action": "stop_drag"}`                   | `StopDrag()` — exit drag/freedrive |
| `{"action": "set_drag_sensitivity", "value": 1..90, "index": 0..6}` | `DragSensivity(index,value)`; `index` 0 = all axes (default), 1..6 = J1..J6; smaller `value` = more resistance |

## Simulated arm

The module also registers a second model, `viam:dobot:cr10a-simulated`, a
hardware-free arm for testing motions, configs, and the 3D scene viewer while
away from a physical controller. It shares the same kinematic model and link
meshes as `viam:dobot:cr10a`, so `Kinematics`, `Geometries`, and `Get3DModels`
return identical data. There is no `host`/port — joint motion is interpolated
in software against a realtime clock, and `MoveToPosition` plans through the
same planner the live arm uses.

Example attributes configuration:

```json
{
  "speed_degs_per_sec": 60,
  "use_urdf": false
}
```

| Attribute        | Type | Default | Description |
|------------------|------|--------:|-------------|
| `speed_degs_per_sec` | number | 60 | Top joint travel speed. All joints scale their speed so a multi-joint move finishes together, matching the planner's interpolation. |
| `simulate_time`  | bool | true | When `true`, a background goroutine advances the arm in real time. Set `false` only for deterministic testing (the arm then holds position until driven manually). |
| `use_urdf`       | bool | false | Same meaning as on the live model: load kinematics + meshes from `arm/cr10a.urdf` instead of the embedded JSON. Requires `VIAM_MODULE_ROOT`. |
| `mesh_decimation_ratios` | array of numbers | `[0.1, …]` (7 entries) | Same meaning as on the live model; only used when `use_urdf` is `true`. |

The simulated model supports one `DoCommand`: `{"command": "get_motion_params"}`
returns `{"speed_degs_per_sec": <number>}`.

## Build

```sh
make            # builds bin/viam-cr10a
make module     # builds bin/module.tar.gz for upload via `viam module upload`
make test       # unit tests
```

The build links against `libnlopt` (transitive dep of the Viam motion
planner). On a vanilla build host install `libnlopt-dev` (`apt-get install
libnlopt-dev` on Ubuntu/Debian, `brew install nlopt` on macOS).

## Kinematics model

The shipped kinematics file (`arm/cr10a_kinematics.json`) is converted
directly from Dobot's `cr10_robot.urdf` (the CR10A shares the CR10's
kinematic skeleton; both have a 1.3 m reach and the same DH parameters).
Link lengths in summary:

| Param | Value | Meaning |
|------:|------:|---------|
| d1    | 176.5 mm | base height |
| a2    | 607 mm   | upper arm |
| a3    | 568 mm   | forearm |
| d4    | 191 mm   | wrist offset along forearm |
| d5    | 125 mm   | wrist 1→2 |
| d6    | 108.4 mm | flange to TCP (no tool) |

Joint limits are taken directly from the URDF (`<limit lower/upper>`, in
radians) and stored in the JSON in degrees: `±359.24°` (±6.27 rad) for
J1/J2/J4/J5/J6 and `±163.92°` (±2.861 rad) for J3.
`TestJSONURDFJointLimitsAgree` guards that the two models stay in sync — the
JSON previously shipped placeholder `±180°`/`±360°` limits that disagreed
with the URDF and needlessly forbade valid configurations.

Collision geometry depends on the source. With `use_urdf: true` the planner
uses the decimated link meshes, so collision volumes match the visual model
exactly. The default embedded JSON instead uses one primitive per link —
capsules for the long arm/wrist links and boxes for the near-cubic base and
shoulder housings — each fitted to its link mesh's bounding box.
`TestJSONGeometriesFitMeshes` keeps every primitive centered on and aligned
with its mesh. These primitives are approximations of non-cylindrical links;
use `use_urdf: true` when you need mesh-accurate collision checking.

If you mount a tool, append a fixed `link` between `joint6` and the TCP
in the kinematics file (or override the model via the motion service's
frame system).

## Limitations / not yet implemented

- **Drag-teach mode (partial).** Basic drag/freedrive *is* exposed via the
  `start_drag` / `stop_drag` / `set_drag_sensitivity` DoCommand actions
  (`StartDrag()` / `StopDrag()` / `DragSensivity(index,value)`); the controller
  refuses to enter drag while an alarm is latched, so `clear_error` first. What
  is **not** implemented are the per-style "alldrag / gesture / translation"
  drag variants — those are a DobotStudioPro HTTP-API feature, not part of the
  ASCII protocol this module speaks.
- **No tool / user frames.** Calls assume tool=0, user=0. If you need to
  drive a non-base coord system, set it once via DoCommand passthrough
  (you can extend the switch in `cr10a.go`).
- **`MovL` not used.** All Cartesian moves go through Viam's planner,
  which produces a joint trajectory; the driver never sends `MovL`. If
  you specifically need on-controller linear interpolation, add a
  DoCommand action.
- **`Get3DModels` returns the link meshes as GLB.** It serves the 7
  bundled CR10A meshes as binary glTF (`model/gltf-binary`) from
  `arm/3d_models/cr10/`, keyed to the active kinematic model's frame
  names, regardless of `use_urdf` (the JSON and URDF link frames
  coincide, guarded by `TestPerLinkFrameAlignment`). GLB is used because
  the Viam app's 3D scene viewer renders glTF, not PLY. The GLBs are
  committed, pre-converted from the source STLs (which still back
  collision geometry). It needs `VIAM_MODULE_ROOT` set to find the
  meshes; if unset it logs a warning and returns an empty map.
  Known issue: with the default JSON kinematics the visual meshes do not
  line up with the SVA collision geometry in the 3D scene viewer (RDK
  attaches SVA link collision at the head of the link transform while the
  mesh renders at the tail); URDF-mode collision does line up. This is
  unresolved.
- **CR10A vs CR10.** The URDF is labeled `cr10_robot`. CR10A is the "A"
  refresh of CR10 and shares the same link geometry but has updated
  joint hardware. The kinematic chain is identical for motion planning
  purposes.

## Hardware bring-up checklist

1. Wire the controller to the same network as your viam-server host.
2. Confirm the CR10A web UI is reachable at `http://<host>` and the arm
   is **not in error state** (red banner). Press the physical reset
   button if needed.
3. Set `host` in your robot config to the controller IP and add the
   component as above.
4. Watch `viam-server` logs on first connect for `EnableRobot ErrorID=…`
   messages — non-zero ErrorIDs surface here. The most common are
   `EnableRobot` failing because the controller hasn't finished `PowerOn`
   (wait ~10s and reconfigure), or `MovJ` failing with ErrorID=22 because
   the target is outside reach.

## References

- [Dobot CR-series TCP/IP protocol](https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html)
- [Dobot-Arm/TCP-IP-Python-V4](https://github.com/Dobot-Arm/TCP-IP-Python-V4) (reference implementation)
- [cr10_robot.urdf](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis/blob/main/dobot_description/urdf/cr10_robot.urdf) (kinematics source)
- [Viam arm component docs](https://docs.viam.com/operate/reference/components/arm/)
