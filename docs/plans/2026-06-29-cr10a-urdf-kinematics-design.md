# Optional URDF + Mesh Kinematics for `cr10a`

**Date:** 2026-06-29
**Status:** Design approved, pending implementation plan

## Goal

Add the manufacturer URDF and meshes (from the Dobot ROS2 V4 driver) to this
module as an **optional** source of kinematics and visualization for the
existing `cr10a` model. Today the model loads a hand-authored
`cr10a_kinematics.json` with capsule collision primitives and no visual
geometry — fine for the planner, plain in the Viam app's 3D view and a
lower-fidelity collision model.

Scope is **cr10a only**. The `cr10af` URDF/meshes exist in the ROS2 driver and
the design is kept factored so adding `cr10af` later is a small follow-up, but
it is out of scope here.

## Reference patterns surveyed

- **Current dobot module** (`arm/cr10a.go`): single model, kinematics via
  `//go:embed cr10a_kinematics.json` + `referenceframe.UnmarshalModelJSON` in
  `Reconfigure`. Wire protocol in `client.go`/`feedback.go` is degrees/mm;
  Cartesian moves go through the planner and reach the controller only as
  joint-space `MovJ`.
- **xarm module** (`../viam-ufactory-xarm/arm/xarm.go`): per-model embedded JSON
  *plus* optional URDFs. A `use_urdfs` config bool flips `MakeModelFrame` to
  `referenceframe.ParseModelXMLFile(<VIAM_MODULE_ROOT>/arm/<model>.urdf, name,
  meshDecimationRatios)`. URDFs reference meshes as
  `package://<pkg>/meshes/.../link.stl`; Viam strips `package://<pkg>/` and
  resolves the remainder relative to the URDF file's directory. STLs live in
  `arm/meshes/`. `make module` tars `arm/meshes arm/*.urdf` into the bundle.
  Meshes loaded from disk at runtime via `VIAM_MODULE_ROOT`, not embedded.
  `mesh_decimation_ratios` (default 0.1/joint) shrinks heavy meshes for planning.
- **ROS2 driver assets** (`../DOBOT_6Axis_ROS2_V4`):
  `dobot_rviz/urdf/cr10_robot.urdf` (SolidWorks export, full inertial/visual/
  collision per link, `revolute` joints with `limit lower/upper` in radians) and
  `dobot_rviz/meshes/cr10/{base_link,Link1..6}.STL` (~2.7 MB). Mesh refs are
  `package://dobot_rviz/meshes/cr10/...` — same shape xarm's loader handles.

## Decisions

| Decision | Choice |
|---|---|
| Scope | `cr10a` only; keep code factored for a later `cr10af` |
| Selection mechanism | `use_urdf` config bool, xarm-style; embedded JSON is the default until URDF is hardware-validated |
| Mesh packaging | On-disk in `arm/meshes/cr10/`, loaded via `VIAM_MODULE_ROOT`; not embedded |
| Mesh path strings | Keep the URDF's `package://` refs unmodified; rely on Viam's relative resolution |
| FK-equivalence test | Assert with **loose** tolerance (catch gross frame errors, allow mesh-origin slack) |
| Source of truth | URDF becomes authoritative for kinematics + collision **once validated**; JSON is the legacy fallback. Default flips to URDF in a follow-up after a hardware check. |

## Architecture

The embedded `cr10a_kinematics.json` stays the default model source. When
`use_urdf` is set, the model-frame source swaps to the ROS2-derived URDF, which
carries full visual + collision meshes. The wire protocol, unit conversion, and
joint-space `MovJ` path are untouched — the URDF only changes the
`referenceframe.Model` returned by `Kinematics()` (consumed by the planner and
the Viam app's 3D view).

## Files

1. **`arm/cr10a.urdf`** — copied from
   `DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/cr10_robot.urdf`. Mesh refs left as-is
   (`package://dobot_rviz/meshes/cr10/base_link.STL`); Viam strips
   `package://dobot_rviz/` and resolves `meshes/cr10/...` relative to `arm/`.
2. **`arm/meshes/cr10/*.STL`** — copied from the ROS2 driver (7 STLs, ~2.7 MB).
3. **`arm/cr10a.go`**
   - `Config` gains `UseURDF *bool` and `MeshDecimationRatios []float64`.
   - `Validate` checks each ratio is in `[0,1]`; defaults to `0.1` per joint when
     URDF is on and none given (xarm behavior).
   - New helper `makeModelFrame(conf, name)`: if `UseURDF` →
     `referenceframe.ParseModelXMLFile(filepath.Join(os.Getenv("VIAM_MODULE_ROOT"),
     "arm/cr10a.urdf"), name, ratios)`; else current
     `UnmarshalModelJSON(cr10aKinematicsJSON, name)`.
   - `Reconfigure` calls `makeModelFrame` instead of unmarshalling inline.
4. **`Makefile`** — `make module` tar line extended with
   `arm/cr10a.urdf arm/meshes`.

`meta.json` is unchanged (build stays `make module`).

## Data flow

`Reconfigure` → `makeModelFrame(conf)` → `a.model` → `Kinematics()` → planner /
Viam app 3D view. Motion commands are unaffected: Cartesian still delegates to
`armplanning.MoveArm` → `MoveToJointPositions` → joint-space `MovJ` in degrees.

## Error handling

- `use_urdf` on but `VIAM_MODULE_ROOT` unset, or URDF/mesh files missing →
  `ParseModelXMLFile` errors → `Reconfigure` fails loudly with a clear message.
  **No silent fallback to JSON** — a broken visualization config must be visible.
- `mesh_decimation_ratios` out of range → config-validation error.

## Testing

- **`TestURDFParse`** — mirrors `TestKinematicsParse`: the URDF parses, has 6
  DoF, zero pose is finite and within reach. Resolves the URDF via a path that
  works in CI (relative to the package dir).
- **FK-equivalence test** — sample several joint configs, run FK through both the
  JSON model and the URDF model, assert TCP poses agree within a **loose**
  tolerance. Runs in CI without hardware. May fail on first write — that is the
  point: it surfaces frame discrepancies (base z-offset, the URDF's
  `dummy_link`/`dummy_joint`, the Link6 flange/TCP) before they silently move the
  tool when `use_urdf` is toggled.
- Existing `client_test.go` / feedback / parse tests unchanged.

## Known risks / follow-ups

- The JSON and URDF were authored independently; the FK-equivalence test is the
  guard. Reconcile the URDF to be physically correct as the future source of
  truth.
- The URDF's `±6.27 rad` joint limits are wider than physically safe and wider
  than the JSON; tighten to the real CR10A limits during reconciliation.
- URDF collision geometry is full meshes → slower planning than the capsules;
  `mesh_decimation_ratios` (default 0.1) mitigates.
- After a hardware bring-up check confirms the URDF, flip the default so URDF is
  the standard kinematics source (separate change).
- `cr10af`: same mechanism, second URDF + mesh set — a small follow-up.
