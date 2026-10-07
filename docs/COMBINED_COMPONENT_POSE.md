# Combined component rigid pose

ShipQA now has a controlled validation path for recovering component translation
and rotation from the same local rigid transform. This closes the unsafe gap where
the earlier percentile-envelope translation method became invalid when a component
was rotated.

## Engineering method

After surface-deviation detection and component identification, the engine isolates
the detected component's nominal grouped OBJ faces and the corresponding observed
scan region. Robust point-to-plane ICP estimates one nominal-to-observed 4x4 rigid
transform. The engine derives both quantities from that transform:

- signed XYZ Euler rotation in degrees using the established `Rz * Ry * Rx`
  convention;
- movement of the nominal component center in millimetres.

Translation axes are reported only when the nominal surfaces contain enough
normal-facing measurement support. For the current 4000 mm STF_03 geometry, Y and
Z are observable while longitudinal X remains unavailable without sufficient end
surface evidence. The raw center movement remains in the prediction for diagnosis,
but an unsupported axis is null in the engineering estimate.

The local quality gate remains unchanged:

- final local ICP fitness at least 0.95;
- final local ICP RMSE at most 5 mm.

The 5 mm defect gate, 30,000 point sampling, 0.35 mm noise, seed 42, component
identification, and ICP settings were not changed for this milestone.

## Controlled Rhino sweep

Run `rhino_scripts/09_generate_combined_pose_validation_batch.py` in Rhino 7 with
`RunPythonScript`. It creates eleven independent cases:

- one zero-defect control;
- six Y/Z translation plus single-axis rotation cases;
- two cases that also characterize the known longitudinal X limitation;
- two Y/Z translation plus multi-axis rotation cases.

Every case deletes the previous as-built copy and rebuilds from nominal geometry.
Each scan and ground-truth record has a unique case directory. Sampling remains
sequential. Ground truth contains the injected rigid transform, but the prediction
engine receives only the scan, nominal OBJ, component manifest, and output path.

After Rhino reports the completed run directory, execute:

```powershell
.\.venv\Scripts\python.exe -B src\validate_combined_poses.py `
  data\validation\<combined_pose_run> `
  --output-dir data\validation\<combined_pose_run>_analysis
```

The runner completes every prediction before opening any ground-truth file. Its
CSV and aggregate JSON report detection, component identity, per-axis translation
errors with explicit denominators, rotation errors, joint-pose availability,
quality metrics, and zero-control false positives.

## Current evidence and limit

The geometric primitive is covered by an exact synthetic component test containing
simultaneous XYZ translation and XYZ rotation. On existing STF_03 data, the grouped
pure -8 mm Y case produces a local-rigid Y estimate of -7.986415 mm and a rotation
of 0.006572 degrees. Three existing pure-rotation cases keep inferred Y/Z center
movement within 0.015 mm. The archived legacy -8 mm regression remains
-8.126176 mm, and all 62 automated tests pass.

## Completed sweep result

Rhino run:

`data/validation/combined_pose_20261007T153550Z_378d6b84`

All ten non-zero cases were detected, all ten identified STF_03, and all ten
produced a quality-gated Y/Z-plus-rotation pose. The zero control produced no raw or
supported candidates and no false component finding. There were no execution
failures.

| Case | Injected translation XYZ mm | Predicted XYZ mm | Injected rotation XYZ deg | Predicted rotation XYZ deg | Angular error deg | Candidates |
|---|---:|---:|---:|---:|---:|---:|
| 002 | 0, -16, 8 | null, -15.994641, 8.012481 | 0, 0, 0.75 | -0.013888, 0.000227, 0.749683 | 0.013893 | 1214 |
| 003 | 0, 16, -8 | null, 16.009386, -7.993319 | 0, 0, -0.75 | -0.020183, 0.000208, -0.750529 | 0.020191 | 1090 |
| 004 | 0, -8, -12 | null, -7.989817, -11.986839 | 0, 0.75, 0 | -0.013219, 0.750379, -0.000429 | 0.013226 | 970 |
| 005 | 0, 8, 12 | null, 8.011895, 12.003743 | 0, -0.75, 0 | -0.011982, -0.749994, -0.000338 | 0.011991 | 1061 |
| 006 | 0, -8, 4 | null, -7.988491, 4.013394 | 5, 0, 0 | 4.988021, 0.000236, -0.000275 | 0.011984 | 628 |
| 007 | 0, 8, -4 | null, 8.011276, -3.992558 | -5, 0, 0 | -5.015132, 0.000124, -0.000487 | 0.015140 | 636 |
| 008 | 8, -8, 4 | null, -7.993001, 4.010536 | 0, 0, 0.75 | -0.010343, 0.000244, 0.749632 | 0.010353 | 836 |
| 009 | -8, 8, -4 | null, 8.012757, -3.994644 | 0, -0.75, 0 | -0.010851, -0.749987, -0.000469 | 0.010867 | 945 |
| 010 | 0, -12, 8 | null, -11.991187, 8.007009 | 2, -0.5, 0.5 | 1.991810, -0.499946, 0.499652 | 0.008200 | 1139 |
| 011 | 0, 12, -8 | null, 12.015644, -7.988645 | -2, 0.5, -0.5 | -2.008921, 0.500343, -0.500507 | 0.008937 | 1045 |

Aggregate translation results use explicit denominators:

- Y: 10/10 estimates, +0.010382 mm bias, 0.010382 mm MAE, 0.015644 mm maximum absolute error;
- Z: 10/10 estimates, +0.009116 mm bias, 0.009116 mm MAE, 0.013394 mm maximum absolute error;
- X characterization: 0/2 estimates because the long component has insufficient longitudinal surface support.

Rotation was valid in 10/10 cases. Mean total angular error was 0.012478 degrees
and maximum total angular error was 0.020191 degrees. Final local fitness was
0.999449--0.999498 and RMSE was 3.447--3.459 mm.

All Y and Z errors have a small positive sign, and RX has a small negative bias.
Opposite injected directions remain similar in magnitude, so this run does not show
a concerning sign-dependent failure. The small systematic terms should be tracked
on real and independently measured data rather than corrected from one synthetic
sampling realization.

This qualifies combined pose only for the present STF_03 synthetic geometry and
sampling model. It does not qualify arbitrary ship components or real yard scans.
