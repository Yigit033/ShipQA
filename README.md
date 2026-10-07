# ShipQA

ShipQA is an engineering prototype for dimensional quality assurance of ship
structures. It compares nominal CAD geometry with an as-built point cloud and
turns geometric differences into traceable component-level findings.

The currently validated product question is intentionally narrow:

> Can the system align a scanned stiffened panel to nominal CAD, identify an
> incorrectly positioned stiffener, and independently estimate its rigid pose?

For the controlled STF_03 experiments, the answer is yes across positive and
negative Y translations, several global scan poses, partial coverage with survey
control, isolated environmental outliers, and local component rotations. The joint
translation-plus-rotation range sweep is ready for Rhino validation. This is not
yet a claim of general shipyard or full-vessel readiness.

## What works today

- Rhino 7 generation of a nominal plate with five T-stiffeners
- controlled as-built STF_03 translations
- reproducible 30,000-point synthetic scans with 0.35 mm Gaussian noise
- nominal OBJ and component-manifest export
- independently verified float64 point-to-triangle surface distances
- global rigid scan-to-CAD registration
- coarse alignment from a survey/scanner transform or corresponding points
- registration quality gating before component QA
- isolated-point filtering and spatial support for defect candidates
- component identification and Y-displacement estimation
- quality-gated local component rotation estimation
- joint component translation and rotation from one local rigid transform
- immutable ground-truth validation runs and structured JSON/CSV output

Current controlled results include:

- six out of six detectable Y-translation cases identified STF_03,
- no false positive in the zero-defect control,
- registration acceptance behaviour correct in 24 out of 24 robustness cases,
- twelve out of twelve detectable robustness cases correctly identified,
- unsafe partial or badly oriented registrations rejected before QA,
- sixty-two automated tests passing.

See [registration robustness](docs/REGISTRATION_ROBUSTNESS.md),
[surface-distance investigation](docs/SURFACE_DISTANCE_INVESTIGATION.md), and
[combined component pose](docs/COMBINED_COMPONENT_POSE.md) for evidence and
limitations.

## Engineering pipeline

```text
Nominal CAD / production model
              +
As-built scan / point cloud
              |
              v
      coarse scan alignment
              |
              v
       robust fine registration
              |
              v
      registration quality gate
              |
              v
    point-to-mesh surface deviation
              |
              v
 supported defect-region detection
              |
              v
       component identification
              |
              v
 component displacement estimation
              |
              v
 joint translation + rotation estimation
```

Nominal geometry, as-built geometry, scan data, surface deviation, and component
pose deviation are treated as separate engineering concepts. Ground-truth defect
metadata is read only after predictions have completed.

## Repository layout

```text
ShipQA/
|-- rhino_scripts/   Rhino 7 / IronPython 2.7 CAD and scan generation
|-- src/             Python 3.12 geometry, registration, QA, and validation
|-- tests/           numerical, regression, isolation, and interface tests
|-- docs/            experiment evidence, assumptions, and limitations
|-- data/            compact reference and regression fixtures
|-- AGENTS.md        project engineering rules
`-- requirements.txt
```

Generated registration runs and analysis outputs are intentionally excluded from
Git. The harnesses recreate them in unique directories without overwriting prior
runs.

## Requirements

### External analysis engine

- Windows development environment currently validated
- Python 3.12
- NumPy 2.5.3
- Open3D 0.20.0

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

### Rhino automation

- Rhino 7
- IronPython 2.7
- RhinoCommon and `rhinoscriptsyntax`

The files under `rhino_scripts/` intentionally avoid Python 3-only syntax and
third-party pip dependencies.

## Run the tests

```powershell
.\.venv\Scripts\python.exe -B -W ignore::DeprecationWarning `
  -m unittest discover -s tests -v
```

## Run one QA prediction

Without registration, for a scan already expressed in CAD coordinates:

```powershell
.\.venv\Scripts\python.exe -B src\qa_engine.py `
  --scan data\synthetic_scan_v1.xyz `
  --mesh data\nominal_reference.obj `
  --manifest data\component_manifest.json `
  --output-dir data\analysis\example_aligned
```

With global rigid registration:

```powershell
.\.venv\Scripts\python.exe -B src\qa_engine.py `
  --scan <scan.xyz> `
  --mesh <nominal_reference.obj> `
  --manifest <component_manifest.json> `
  --output-dir <new-output-directory> `
  --register
```

For partial or arbitrarily oriented scans, provide field control:

```powershell
  --initial-alignment <initial_alignment.json>
```

For scans contaminated with isolated environmental returns:

```powershell
  --filter-isolated-outliers
```

The output directory contains `prediction.json`, `analysis.log`,
`defect_candidates.xyz`, and, when registration is enabled,
`registered_scan.xyz`. Existing output directories are never replaced.

## Coarse-alignment input

A scanner/survey transform uses millimetres and maps raw scan coordinates to CAD:

```json
{
  "schema_version": 1,
  "units": "mm",
  "method": "transform_matrix",
  "source": "survey_network",
  "transform_scan_to_cad": [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ]
}
```

Alternatively, use `method: "corresponding_points"` with matching
`scan_points_mm` and `cad_points_mm` arrays containing at least three non-collinear
points. The engine reports correspondence residuals and refines the coarse result
with robust ICP.

## Rhino controlled workflow

Run Rhino scripts through `RunPythonScript`:

1. `01_build_nominal_panel.py`
2. `02_build_asbuilt_test.py`
3. `03_generate_synthetic_scan.py`
4. `04_export_nominal_mesh.py`
5. `05_export_component_manifest.py`

`06_generate_y_validation_batch.py` generates the complete sequential Y-translation
sweep. Every case rebuilds from nominal geometry and keeps its own scan and ground
truth.

`09_generate_combined_pose_validation_batch.py` generates the controlled joint
translation-plus-rotation sweep. It also rebuilds every case from nominal geometry.

## Validation commands

Evaluate a completed Rhino Y sweep:

```powershell
.\.venv\Scripts\python.exe -B src\validate_y_translations.py <run-directory> `
  --output-dir <new-analysis-directory>
```

Generate and evaluate the focused registration robustness matrix:

```powershell
.\.venv\Scripts\python.exe -B src\generate_registration_robustness.py `
  <completed-y-run> <new-registration-run>

.\.venv\Scripts\python.exe -B src\validate_registration_robustness.py `
  <new-registration-run> <new-analysis-directory>
```

All predictions complete before validation code reads ground truth.

Evaluate a completed Rhino combined-pose sweep:

```powershell
.\.venv\Scripts\python.exe -B src\validate_combined_poses.py `
  <combined-pose-run-directory> --output-dir <new-analysis-directory>
```

## Current limits

ShipQA has not yet been qualified for real yard scans. Important remaining work
includes:

- displacement-estimator stability under changing and partial coverage,
- coherent clutter attached to nearby structures,
- occlusion and varying scan density,
- multi-station scan registration,
- survey-network and scanner systematic errors,
- combined-pose range qualification, deformation, missing components, and multiple defects,
- validation on real point-cloud data.

Registration thresholds currently reflect the controlled 0.35 mm-noise prototype;
they are not shipyard acceptance tolerances. Classification-society, yard, project,
and component tolerances must be handled as explicit engineering inputs in later
milestones.

## Design principles

- Deterministic geometry performs dimensional measurement.
- All numerical quantities are explicit in millimetres.
- Registration uncertainty is kept separate from manufacturing deviation.
- A failed registration cannot produce a component QA conclusion.
- Raw evidence and filtered engineering candidates are both reported.
- Generated data stays separate from source code.
- AI/ML is introduced only where deterministic geometry becomes inadequate.

The objective is a technically defensible inspection tool that helps engineers make
decisions, not a visualization demo.
