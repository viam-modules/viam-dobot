# viam-dobot-cr10a

A Viam **arm** module for the Dobot CR10A 6-axis collaborative robot. The
driver speaks the native CR-series TCP/IP remote-control protocol — no
DobotStudio or ROS bridge in the loop — and ships a Viam kinematics model
derived from Dobot's official URDF, so the motion service can plan paths,
check collisions, and inverse-kinematic against the real link geometry.

Registered model: `viam-soleng:dobot:cr10a`.

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

Add to the `services` / `components` section of your robot config:

```json
{
  "name": "cr10a",
  "model": "viam-soleng:dobot:cr10a",
  "type": "arm",
  "namespace": "rdk",
  "attributes": {
    "host": "192.168.1.6",
    "speed_factor": 30,
    "joint_speed":  50,
    "joint_accel":  50,
    "auto_enable":  true
  }
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

## DoCommand actions

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

Joint limits default to `±180°` for J1/J2/J4/J5, `±163.9°` for J3, and
`±360°` for J6 — the conservative URDF-default values. The CR10A hardware
can in fact rotate further on most joints; widen the `min`/`max` in the
JSON if your application needs it.

If you mount a tool, append a fixed `link` between `joint6` and the TCP
in the kinematics file (or override the model via the motion service's
frame system).

## Limitations / not yet implemented

- **No drag-teach mode.** `RobotMode()` reports drag as mode 6/8, but the
  driver doesn't expose a way to enter it. Easy to add via DoCommand if
  needed.
- **No tool / user frames.** Calls assume tool=0, user=0. If you need to
  drive a non-base coord system, set it once via DoCommand passthrough
  (you can extend the switch in `cr10a.go`).
- **`MovL` not used.** All Cartesian moves go through Viam's planner,
  which produces a joint trajectory; the driver never sends `MovL`. If
  you specifically need on-controller linear interpolation, add a
  DoCommand action.
- **Get3DModels returns empty.** Collision geometry is the cylinders in
  the kinematics JSON; we don't ship STL meshes.
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
