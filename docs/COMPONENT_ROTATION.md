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

The result is currently `estimated_provisional`. Field acceptance thresholds will
be defined from the controlled Rhino sweep rather than selected in advance to make
the results pass.

## Current evidence

All 54 tests pass. A known asymmetric component rotation is recovered below 0.1
degree angular error. On a previous pure Y=-8 mm translation scan, the estimator
returns only 0.01365 degrees rotation, providing an initial translation/rotation
separation check. The preserved historical Y regression remains unchanged.

The Rhino rotation batch has not yet been generated, so no controlled STF_03
rotation accuracy claim is made.

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
