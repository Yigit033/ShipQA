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

The eleven-case Rhino sweep has not yet been generated or evaluated. Therefore the
combined-pose capability is implemented and regression checked, but it is not yet
qualified across the planned STF_03 range. It is also not evidence of readiness for
arbitrary ship components or real yard scans.
