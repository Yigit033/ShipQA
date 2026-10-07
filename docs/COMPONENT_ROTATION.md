# Controlled local component rotation hardening

## Purpose

This phase tests whether ShipQA can distinguish and recover a rigid rotation of a
single stiffener while the nominal panel and all other components remain fixed.
Global scan-to-CAD registration and local component rotation remain separate.

The prediction engine receives only the scan, grouped nominal OBJ, component
manifest and output location. Expected component identity, changed axis and known
angles remain in ground truth until all predictions finish.

## Controlled Rhino matrix

`08_generate_rotation_validation_batch.py` rebuilds every case from nominal CAD,
rotates the complete STF_03 assembly about its nominal center
`[2000, 1200, 92] mm`, then samples 30,000 points with sigma 0.35 mm and seed 42.

- RX: -5, -1.5, +1.5, +5 degrees
- RY: -0.75, -0.25, +0.25, +0.75 degrees
- RZ: -0.75, -0.25, +0.25, +0.75 degrees
- one zero control

The +/-1.5 degree RX cases are explicit threshold characterization: the maximum
cross-section movement can remain below the unchanged 5 mm surface gate. Missing
detection is recorded, not silently counted as an estimator failure.

## External estimator

After the existing detector identifies a component, ShipQA selects a broad local
component region and performs deterministic, robust point-to-plane ICP against the
component's grouped nominal faces. The output includes:

- nominal-to-observed 4x4 local transform
- 3x3 rotation matrix
- signed XYZ Euler angles using the `Rz * Ry * Rx` convention
- component-center translation induced by the local transform
- point counts, stage fitness and inlier RMSE
- estimator source hash

Controlled acceptance requires final ICP fitness at least 0.95 and inlier RMSE at
most 5 mm. Failed quality returns `rejected_low_quality`. These remain controlled
qualification limits; real-data field limits require real scanner validation.

## Current evidence

All 54 tests pass. A known asymmetric component rotation is recovered below 0.1
degree angular error. On a previous pure Y=-8 mm translation scan, the estimator
returns only 0.01365 degrees rotation, providing an initial translation/rotation
separation check. The preserved historical Y regression remains unchanged.

## Completed sweep - 2026-10-07

Rhino generated all 13 scans in
`component_rotation_20261007T151253Z_92186729`.

The first run exposed two independent limitations. A fixed 3 mm Tukey loss rejected
real coarse inliers displaced by up to 26 mm, producing 0.408 degree mean angular
error. Progressive robust scales of 30, 10 and 3 mm reduced mean angular error to
0.00935 degrees and maximum error to 0.01542 degrees.

RX +/-5 degree roll produced 161--164 valid threshold points distributed along the
4000 mm stiffener, so isotropic 100 mm DBSCAN rejected them as locally sparse. A
manifest-aware fallback scales only the component's longest axis before applying
the unchanged 100 mm / 10 point support criteria. Both roll cases are now detected
without changing the 5 mm surface gate. The earlier 24-case registration/outlier
matrix retained 24/24 correct acceptance decisions and zero false positives in all
six accepted controls.

Final results:

- zero-control false positives: 0/1
- detectable rotations detected: 10/10
- correct component identifications: 10/10
- valid quality-gated rotation estimates: 10/10
- mean / maximum total angular error: 0.00935 / 0.01542 degrees
- RX bias / MAE / maximum target-axis error: -0.00642 / 0.00642 / 0.00949 degrees
- RY bias / MAE / maximum target-axis error: 0.000225 / 0.000225 / 0.000417 degrees
- RZ bias / MAE / maximum target-axis error: -0.000336 / 0.000336 / 0.000347 degrees
- RX +/-1.5 degree threshold characterization: 0/2 detected, as expected
- execution failures: 0

All 55 tests pass. The final analysis is preserved under
`component_rotation_20261007T151253Z_92186729_final`.

## Run sequence

In Rhino 7, open a copy of the nominal panel document and run:

```text
rhino_scripts/08_generate_rotation_validation_batch.py
```

After generation, run externally with the project environment:

```powershell
.\.venv\Scripts\python.exe src\validate_component_rotations.py `
  data\validation\component_rotation_<run_id>
```
