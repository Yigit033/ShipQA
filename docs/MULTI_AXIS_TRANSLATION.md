# Multi-axis component translation hardening

This phase extends the controlled STF_03 experiment from Y-only displacement to
independent X, Y and Z translations. It does not change the 5 mm detection gate,
component identification, the validated Y percentile estimator, scan density,
noise, seed or registration implementation.

The engine is not told which axis changed. Ground truth is read by the evaluator
only after every prediction has completed.

## Controlled matrix

The Rhino batch contains one zero control and four detectable magnitudes on each
axis: -16, -8, +8 and +16 mm. Every case is rebuilt from nominal geometry. Every
scan contains 30,000 points, uses 0.35 mm Gaussian coordinate noise and seed 42.

## Nominal component identity

New nominal OBJ exports contain `g PART_ID` records. Open3D still reads the OBJ as
one global surface. The pose module uses the same file to select only the nominal
faces listed in the detected component's manifest entry. Historical OBJ fixtures
have no group records; they retain the validated Y result and explicitly report
that XYZ pose is unavailable.

## Estimation and limitations

The existing Y estimate is preserved exactly. X and Z use the midpoint of the 2nd
and 98th percentile envelopes of measured component-region points relative to a
deterministic area sample of the grouped nominal component surface.

- A 4000 mm prismatic stiffener is weakly observable in X because longitudinal
  translation is mainly visible at its ends.
- Negative Z translation can make the controlled stiffener intersect the base
  plate. It is a numerical stress case, not a physically valid fabrication state.

These cases must be reported as failures or low-observability cases if the
evidence does not support a reliable estimate.

## Run sequence

In Rhino 7, open a copy of the nominal panel document and run:

```text
rhino_scripts/07_generate_xyz_translation_batch.py
```

After generation completes, run:

```powershell
.\.venv\Scripts\python.exe src\validate_xyz_translations.py `
  data\validation\xyz_translation_<run_id>
```

The evaluator writes `summary.csv` and `aggregate.json`, separating detection,
component identification and pose estimation with complete denominators.

## Current evidence

Implementation tests initially passed 48/48, including the preserved -8.126176 mm regression,
ground-truth isolation, XYZ scoring, explicit failure denominators, IronPython 2.7
grammar and a known component-surface translation recovery test.

## Completed sweep - 2026-10-07

Rhino generated all 13 scans in
`xyz_translation_20261007T141807Z_e1b7d7f9`. The first run exposed two unsafe
behaviours: an unobservable X estimate appeared as a false -9 to -12 mm offset,
and fixed low-Z cropping caused 8 to 13 mm errors for negative Z translations.

The corrected estimator now uses nominal face normals and surface area to estimate
the number of scan points supporting each translation axis. An axis requires at
least 10 expected supporting points on faces whose absolute normal component is at
least 0.95. This is geometric evidence rather than an inferred case label or a
ground-truth-dependent confidence score.

For STF_03, X-normal faces are only 0.2012% of the component surface and provide an
expected 3.85 points in a typical 1,915-point component selection. X is therefore
reported as `null` with `insufficient_geometric_support`. Y and Z have approximately
1,233 and 678 expected supporting points respectively.

The final 13-case result is:

- zero-control false positives: 0/1
- Y detection / correct identification / valid estimate: 4/4, 4/4, 4/4
- Y bias / MAE / maximum error: 0.382362 / 0.382362 / 0.382362 mm
- Z detection / correct identification / valid estimate: 4/4, 4/4, 4/4
- Z bias / MAE / maximum error: 0.371843 / 0.371843 / 0.381892 mm
- X detection and estimate: 0/4; all four cases had only 2--8 raw endpoint
  candidates and no supported cluster
- execution failures: 0

Z is measured from the upper component-surface percentile, which remains visible
when the lower region intersects or is obscured by the plate. The validated legacy
Y result remains unchanged. Full XYZ vector error is deliberately not reported
when any axis is unobservable.

The final preserved analysis is
`xyz_translation_20261007T141807Z_e1b7d7f9_axis_reliability_final_v2`. All 49 tests
pass. Rotation hardening has not started.
