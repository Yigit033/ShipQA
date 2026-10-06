# Controlled global registration validation

> Historical first-registration result. The current engine adds hard QA gating,
> field coarse-alignment inputs, isolated-point filtering, and spatial candidate
> support. See [REGISTRATION_ROBUSTNESS.md](REGISTRATION_ROBUSTNESS.md).

## Purpose and scope

This milestone tests whether ShipQA can remove one global rigid coordinate-frame
offset before measuring a local manufacturing deviation. A single transform is
estimated from scan and nominal mesh geometry only. Component IDs, injected
STF_03 shifts, and the injected global transform are not registration inputs.

This first implementation assumes that scan and CAD axes are already roughly
parallel and that most of the panel is visible. It is not an arbitrary-pose,
partial-scan, or shipyard-clutter solution. Symmetric structures cannot always
provide a unique absolute orientation without survey control, scanner poses,
fiducials, or engineer-selected correspondences.

## Method

`src/registration.py` performs:

1. deterministic sampling of 100,000 nominal mesh surface points (seed 42),
2. coarse centroid translation while retaining the rough axis prior,
3. robust point-to-plane ICP at 150, 50, and 15 mm correspondence distances,
4. Tukey losses of 10, 5, and 2 mm respectively,
5. rigid-transform integrity checks, and
6. explicit transform and quality metrics in `prediction.json`.

The matrix convention is a column-vector transform from raw scan coordinates to
nominal CAD coordinates. The registered points are then passed into the existing
surface-deviation, 5 mm detection, component identification, and percentile shift
estimation logic. Those algorithms and settings did not change.

`qa_engine.py --register` additionally writes `registered_scan.xyz`, so the exact
geometry used by downstream QA remains inspectable.

The provisional quality checks expose whether fine ICP fitness is at least 0.8,
whether surface P95 improved, and whether post-registration P95 lies inside the
final 15 mm correspondence range. These checks passed this controlled run. They
are deliberately labelled provisional and are not field acceptance criteria.

## Controlled experiment

The source is the immutable nine-case Y-translation run. The external generator
applied the same transform to each completed scan:

- rotations: X = +1.2 degrees, Y = -0.8 degrees, Z = +1.5 degrees,
- rotation center: (2000, 1200, 86) mm,
- translation after rotation: (+75, -45, +30) mm.

No points were resampled, no noise was added, and the local STF_03 defects were not
changed. The known matrix is stored only in each case's `ground_truth` directory.
All nine registered predictions finish before the evaluator reads any ground truth.

Run artifacts are under:

`data/registration_validation/rigid_20261006T114109Z/`

`run/` contains immutable transformed inputs and ground truth. `analysis_final/`
contains predictions and evaluation. The earlier `analysis/` directory is retained
as execution history; `analysis_final/` matches the final source hashes and includes
the provisional quality fields.

## Results

Registration completed for 9/9 cases with no execution failures. All cases passed
the provisional quality checks.

| Metric | Result |
|---|---:|
| Mean rotation error | 0.000185 degrees |
| Maximum rotation error | 0.000210 degrees |
| Mean reference-vertex round-trip RMSE | 0.128933 mm |
| Maximum reference-vertex error over all cases | 0.166715 mm |
| Fine ICP fitness range | 0.947100 to 0.969433 |
| Pre-registration surface P95 range | 65.249 to 67.099 mm |
| Post-registration surface P95 range | 0.678 to 0.993 mm |

Downstream comparison against the already-aligned run:

| Injected Y shift | Aligned prediction | Registered prediction | Registered candidates | Result |
|---:|---:|---:|---:|---|
| -16 | -15.617638 | -15.625925 | 1252 | STF_03 |
| -12 | -11.617638 | -11.625400 | 673 | STF_03 |
| -8 | -7.617638 | -7.624794 | 633 | STF_03 |
| -4 | -3.617638 | -3.625823 | 3 | STF_03 |
| 0 | no estimate | no estimate | 0 | no false positive |
| +4 | +4.382362 | +4.374450 | 1 | STF_03 |
| +8 | +8.382362 | +8.372383 | 608 | STF_03 |
| +12 | +12.382362 | +12.375012 | 643 | STF_03 |
| +16 | +16.382362 | +16.374601 | 1252 | STF_03 |

For the six detectable cases, detection and correct identification are both 6/6,
with 6/6 valid estimates and no exclusions. Mean signed error and MAE are both
0.374313 mm; maximum absolute error is 0.375206 mm. The corresponding aligned-run
values were 0.382362 mm for all three metrics. Registration moved the estimates by
only 0.0072 to 0.0100 mm and did not erase the local manufacturing defect.

The zero control remains clean. At the 5 mm gate, -4 mm changed from one candidate
to three and +8 mm changed from 609 to 608. This is expected sensitivity of points
lying almost exactly on a hard threshold, but it confirms that candidate count alone
is not a stable engineering decision near the gate.

The preserved unregistered -8 mm regression still returns STF_03 and -8.126176 mm.

## Reproduction

From the repository root using `.venv`:

```powershell
.\.venv\Scripts\python.exe -B src\generate_registration_validation.py `
  data\validation\y_translation_20261006T104604Z_b8c0a39c `
  <new-run-directory>

.\.venv\Scripts\python.exe -B src\validate_registration.py `
  <new-run-directory> `
  <new-analysis-directory>
```

Both commands refuse to replace their output directories.

## Remaining risks

This validates one modest global transform on complete synthetic scans. It does
not validate arbitrary initial orientation, partial coverage, occlusion, outliers,
unrelated geometry, repeated-structure ambiguity, scanner scale error, or real scan
systematics. The next registration work should vary transform directions and
magnitudes, then introduce controlled partial scans and outliers. A practical field
workflow also needs an explicit coarse-alignment source such as surveyed targets,
scanner poses, or engineer-selected corresponding points before fine ICP.
