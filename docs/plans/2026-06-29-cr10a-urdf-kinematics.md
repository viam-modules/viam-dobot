# cr10a URDF + Mesh Kinematics Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add the Dobot ROS2 `cr10_robot` URDF and STL meshes to this module as an optional, config-toggled source of kinematics + visualization for the existing `cr10a` model, defaulting to the current embedded JSON until the URDF is hardware-validated.

**Architecture:** A `use_urdf` config bool (xarm-style) selects between `referenceframe.UnmarshalModelJSON(embedded JSON)` and `referenceframe.ParseModelXMLFile(<VIAM_MODULE_ROOT>/arm/cr10a.urdf, …)`. Meshes ship on disk in `arm/meshes/cr10/` and are tarred into the module bundle. The wire protocol and motion path are untouched — only the `referenceframe.Model` returned by `Kinematics()` changes.

**Tech Stack:** Go, `go.viam.com/rdk` v0.123.0 (`referenceframe.ParseModelXMLFile` has the 3-arg `meshDecimationRatios` signature), Dobot ROS2 V4 assets at `../DOBOT_6Axis_ROS2_V4`.

**Design doc:** `docs/plans/2026-06-29-cr10a-urdf-kinematics-design.md`

---

## Task 1: Import URDF + mesh assets

**Files:**
- Create: `arm/cr10a.urdf` (copied)
- Create: `arm/meshes/cr10/{base_link,Link1,Link2,Link3,Link4,Link5,Link6}.STL` (copied)

**Step 1: Copy the assets**

```bash
mkdir -p arm/meshes/cr10
cp ../DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/cr10_robot.urdf arm/cr10a.urdf
cp ../DOBOT_6Axis_ROS2_V4/dobot_rviz/meshes/cr10/*.STL arm/meshes/cr10/
```

**Step 2: Verify the mesh references resolve correctly**

The URDF references meshes as `package://dobot_rviz/meshes/cr10/base_link.STL`.
Viam strips `package://dobot_rviz/` and resolves `meshes/cr10/...` relative to the
URDF's directory (`arm/`). Confirm every referenced file exists:

Run:
```bash
grep -o 'meshes/cr10/[A-Za-z0-9_]*\.STL' arm/cr10a.urdf | sort -u | while read f; do
  test -f "arm/$f" && echo "OK   $f" || echo "MISS $f"
done
```
Expected: all 7 lines start with `OK` (base_link + Link1..6), no `MISS`.

**Step 3: Commit**

```bash
git add arm/cr10a.urdf arm/meshes/cr10
git commit -m "feat: import cr10 URDF and STL meshes from Dobot ROS2 driver"
```

---

## Task 2: Make the module bundle ship the URDF + meshes

**Files:**
- Modify: `Makefile` (the `module` target's `tar` line)

**Step 1: Read the current module target**

Run: `grep -n "tar " Makefile`
Note the existing `tar czf bin/module.tar.gz ...` line.

**Step 2: Add the URDF + meshes to the tar**

Append `arm/cr10a.urdf arm/meshes` to the `tar czf` invocation so the bundle
contains them alongside the binary and `meta.json`. (Matches xarm's
`arm/meshes arm/*.urdf`.)

**Step 3: Verify the tarball contents**

Run:
```bash
make module && tar tzf bin/module.tar.gz | grep -E 'cr10a\.urdf|meshes/cr10'
```
Expected: lists `arm/cr10a.urdf` and all 7 `arm/meshes/cr10/*.STL` entries.

**Step 4: Commit**

```bash
git add Makefile
git commit -m "build: bundle cr10a URDF and meshes in module tarball"
```

---

## Task 3: Add `use_urdf` + `mesh_decimation_ratios` config fields with validation

**Files:**
- Modify: `arm/cr10a.go` (`Config` struct + `Validate`)
- Test: `arm/cr10a_test.go`

**Step 1: Write the failing test**

Add to `arm/cr10a_test.go` (the package's tests use the stdlib `testing`
style — `t.Fatalf`, no `test.That` assertion library):

```go
func TestConfigValidateMeshDecimationRatios(t *testing.T) {
	good := &Config{Host: "1.2.3.4", MeshDecimationRatios: []float64{0, 0.5, 1}}
	if _, _, err := good.Validate("path"); err != nil {
		t.Fatalf("valid ratios rejected: %v", err)
	}
	bad := &Config{Host: "1.2.3.4", MeshDecimationRatios: []float64{1.5}}
	if _, _, err := bad.Validate("path"); err == nil {
		t.Fatalf("expected error for out-of-range ratio, got nil")
	}
}
```

**Step 2: Run it to confirm it fails to compile**

Run: `go test ./arm -run TestConfigValidateMeshDecimationRatios`
Expected: FAIL — `MeshDecimationRatios` is not a field of `Config`.

**Step 3: Add the fields and validation**

In `Config` (after `AutoEnable`):

```go
	// UseURDF loads kinematics + meshes from arm/cr10a.urdf instead of the
	// embedded capsule JSON. Default false (embedded JSON) until the URDF is
	// hardware-validated. Requires VIAM_MODULE_ROOT to be set (it is, under
	// viam-server).
	UseURDF bool `json:"use_urdf,omitempty"`
	// MeshDecimationRatios is the per-joint mesh simplification ratio in [0,1]
	// used when UseURDF is set. Lower = more aggressive. Defaults to 0.1 per
	// joint when empty. Ignored unless UseURDF is true.
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`
```

In `Validate`, before the final `return nil, nil, nil`:

```go
	for i, r := range cfg.MeshDecimationRatios {
		if r < 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratios[%d] must be in [0, 1], got %f", i, r)
		}
	}
```

**Step 4: Run the test to confirm it passes**

Run: `go test ./arm -run TestConfigValidateMeshDecimationRatios -v`
Expected: PASS.

**Step 5: Commit**

```bash
git add arm/cr10a.go arm/cr10a_test.go
git commit -m "feat: add use_urdf and mesh_decimation_ratios config fields"
```

---

## Task 4: Add `makeModelFrame` helper and wire it into `Reconfigure`

**Files:**
- Modify: `arm/cr10a.go` (new helper + `Reconfigure`, imports)
- Test: `arm/cr10a_test.go`

**Step 1: Write the failing test (URDF parses to a valid 6-DoF model)**

`go test` runs with the working directory set to the package dir (`arm/`), so the
URDF resolves via its relative path with no `VIAM_MODULE_ROOT` needed.

Add to `arm/cr10a_test.go` (`referenceframe.Input` is a `float64` alias, so a
zero slice is fine; `pose.Point()` returns an `r3.Vector` with `.X/.Y/.Z`):

```go
func TestURDFParse(t *testing.T) {
	model, err := referenceframe.ParseModelXMLFile("cr10a.urdf", "cr10a", nil)
	if err != nil {
		t.Fatalf("ParseModelXMLFile: %v", err)
	}
	if got := len(model.DoF()); got != 6 {
		t.Fatalf("expected 6 DoF, got %d", got)
	}
	zero := make([]referenceframe.Input, 6)
	pose, err := model.Transform(zero)
	if err != nil {
		t.Fatalf("Transform(zero): %v", err)
	}
	pt := pose.Point()
	if math.IsNaN(pt.X) || math.IsNaN(pt.Y) || math.IsNaN(pt.Z) ||
		math.IsInf(pt.X, 0) || math.IsInf(pt.Y, 0) || math.IsInf(pt.Z, 0) {
		t.Fatalf("non-finite home pose %v", pt)
	}
}
```

**Step 2: Run it**

Run: `go test ./arm -run TestURDFParse -v`
Expected: PASS if the URDF is well-formed and meshes resolve. If it FAILs on a
missing mesh, revisit Task 1 Step 2. (`math` and `referenceframe` are already
imported in the test file.)

**Step 3: Add the `makeModelFrame` helper to `arm/cr10a.go`**

Add near the top-level helpers (and add `"os"` and `"path/filepath"` to imports):

```go
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
		ratios = []float64{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}
	}
	path := filepath.Join(os.Getenv("VIAM_MODULE_ROOT"), "arm", "cr10a.urdf")
	model, err := referenceframe.ParseModelXMLFile(path, name, ratios)
	if err != nil {
		return nil, fmt.Errorf("loading CR10A URDF kinematics from %q: %w", path, err)
	}
	return model, nil
}
```

**Step 4: Replace the inline unmarshal in `Reconfigure`**

In `Reconfigure`, replace the block:

```go
	model, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, conf.ResourceName().ShortName())
	if err != nil {
		return fmt.Errorf("loading CR10A kinematics: %w", err)
	}
```

with:

```go
	model, err := makeModelFrame(newConf, conf.ResourceName().ShortName())
	if err != nil {
		return fmt.Errorf("loading CR10A kinematics: %w", err)
	}
```

**Step 5: Build + run the full package tests**

Run: `go build ./... && go test ./arm -v`
Expected: builds, all tests PASS (default path still uses JSON, so existing
behavior is unchanged).

**Step 6: Commit**

```bash
git add arm/cr10a.go arm/cr10a_test.go
git commit -m "feat: select URDF or JSON kinematics via makeModelFrame"
```

---

## Task 5: FK-equivalence guard test (JSON vs URDF)

This is the safety net from the design: if the two independently-authored models
disagree on the tool frame, toggling `use_urdf` would silently move the TCP.
Loose tolerance — catch gross frame errors, tolerate mesh-origin slack.

**Files:**
- Test: `arm/cr10a_test.go`

**Step 1: Write the test**

Add `"go.viam.com/rdk/spatialmath"` to the test file's imports.
`referenceframe.Input` is a `float64` alias, so the config literals double as
input slices — no conversion helper needed.

```go
func TestJSONURDFForwardKinematicsAgree(t *testing.T) {
	jsonModel, err := referenceframe.UnmarshalModelJSON(cr10aKinematicsJSON, "cr10a")
	if err != nil {
		t.Fatalf("UnmarshalModelJSON: %v", err)
	}
	urdfModel, err := referenceframe.ParseModelXMLFile("cr10a.urdf", "cr10a", nil)
	if err != nil {
		t.Fatalf("ParseModelXMLFile: %v", err)
	}

	// Representative joint configs (radians).
	configs := [][]referenceframe.Input{
		{0, 0, 0, 0, 0, 0},
		{0.5, -0.3, 0.7, 0.2, -0.4, 0.1},
		{-1.2, 0.6, -0.9, 1.0, 0.3, -0.7},
	}
	const posTolMM = 10.0 // loose: analytic links vs mesh origins
	const oriTolDeg = 2.0

	for _, c := range configs {
		jp, err := jsonModel.Transform(c)
		if err != nil {
			t.Fatalf("json Transform %v: %v", c, err)
		}
		up, err := urdfModel.Transform(c)
		if err != nil {
			t.Fatalf("urdf Transform %v: %v", c, err)
		}

		dist := jp.Point().Sub(up.Point()).Norm()
		oriDiff := spatialmath.QuatToR3AA(
			spatialmath.OrientationBetween(jp.Orientation(), up.Orientation()).Quaternion(),
		).Norm() * 180 / math.Pi

		t.Logf("config %v: posΔ=%.2fmm oriΔ=%.2f°", c, dist, oriDiff)
		if dist > posTolMM {
			t.Errorf("config %v: position diff %.2fmm exceeds %.1fmm", c, dist, posTolMM)
		}
		if oriDiff > oriTolDeg {
			t.Errorf("config %v: orientation diff %.2f° exceeds %.1f°", c, oriDiff, oriTolDeg)
		}
	}
}
```

**Step 2: Run it**

Run: `go test ./arm -run TestJSONURDFForwardKinematicsAgree -v`
Expected: PASS — or FAIL reporting the per-config `posΔ`/`oriΔ`.

**Step 3: If it fails — reconcile, do not loosen blindly**

A large `posΔ` (tens of mm or more) means a genuine frame mismatch (base
z-offset, the URDF `dummy_link`/`dummy_joint`, or the Link6 flange/TCP). Per the
design, the **URDF is the intended source of truth**: correct the URDF (or the
JSON) so they agree, rather than inflating the tolerance to hide it. Record the
reconciliation in the design doc's "Known risks" section. Only widen tolerance
for sub-cm differences clearly attributable to mesh-origin vs analytic-link
placement.

**Step 4: Commit**

```bash
git add arm/cr10a_test.go
git commit -m "test: assert JSON and URDF forward kinematics agree"
```

---

## Task 6: Documentation

**Files:**
- Modify: `README.md` (config reference)
- Modify: `CLAUDE.md` (architecture notes)

**Step 1: Document the new config fields in README.md**

Under the configuration section, document `use_urdf` (bool, default false —
loads URDF kinematics + meshes for richer visualization and mesh-accurate
collision; requires the bundled assets) and `mesh_decimation_ratios` (per-joint
`[0,1]`, default 0.1, only used with `use_urdf`).

**Step 2: Add an architecture note to CLAUDE.md**

In the "Non-obvious design decisions" list, note that kinematics has two
sources: the default embedded capsule JSON and an optional URDF
(`arm/cr10a.urdf` + `arm/meshes/cr10/`) selected by `use_urdf`, resolved via
`VIAM_MODULE_ROOT`; that `make module` must bundle the URDF + meshes; and that
`TestJSONURDFForwardKinematicsAgree` guards frame parity between the two.

**Step 3: Verify the build/lint still pass**

Run: `make lint && make test`
Expected: clean vet/gofmt, all tests PASS.

**Step 4: Commit**

```bash
git add README.md CLAUDE.md
git commit -m "docs: document use_urdf kinematics option"
```

---

## Out of scope (follow-ups)

- **Hardware bring-up validation** of the URDF against the real CR10A, then
  flipping the `use_urdf` default to true.
- **Tightening URDF joint limits** from `±6.27 rad` to the real CR10A limits.
- **`cr10af`** as a second model/asset set — same mechanism.

## Verification checklist (final)

- `make test` — all green, including `TestURDFParse` and
  `TestJSONURDFForwardKinematicsAgree`.
- `make module && tar tzf bin/module.tar.gz | grep -E 'cr10a\.urdf|meshes/cr10'`
  — assets present in the bundle.
- Default config (no `use_urdf`) loads the JSON model exactly as before.
