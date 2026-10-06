# Registration robustness milestone

## Engineering objective

This milestone extends the first controlled registration result without changing
the 5 mm surface-deviation gate or the percentile displacement estimator. It tests
the minimum capabilities needed before field scans are credible inputs:

- different rigid-transform directions and magnitudes,
- partial coverage,
- isolated environmental points,
- rejection of untrustworthy registration before component QA, and
- coarse alignment from survey/scanner transforms or corresponding points.

The frozen Rhino scans remain the local manufacturing state. External scripts alter
only the scan coordinate frame and controlled scan visibility/contamination. The QA
engine receives no injected transform or expected component result.

## Product behavior

`qa_engine.py --register` now accepts two optional field inputs:

```text
--initial-alignment <json>
--filter-isolated-outliers
```

Initial-alignment JSON supports:

1. `transform_matrix`: a scanner-pose or survey-derived 4x4 scan-to-CAD matrix.
2. `corresponding_points`: at least three non-collinear scan/CAD point pairs,
   suitable for surveyed targets or engineer-selected correspondences.

Correspondence alignment uses a rigid SVD solution. Reflections, non-rigid matrices,
non-finite values, invalid units, and collinear control points are rejected.

The isolated-point option requires at least four points within a 50 mm radius. It
records the input, retained, and removed counts. It does not silently change the
underlying scan file.

## Registration quality gate

Component detection and displacement estimation run only when all applicable checks
pass:

- fine ICP fitness >= 0.85,
- exact post-registration surface P95 <= 2.0 mm,
- at least 75% of input points retained after optional outlier filtering,
- without external coarse control, registered span ratios of at least
  X/Y/Z = 0.60/0.60/0.50 relative to the nominal model.

Failure produces `registration_rejected`, leaves component and displacement fields
empty, writes the transform and quality evidence, and returns CLI exit code 2. These
limits are validated for the present 0.35 mm-noise prototype. They must be qualified
against scanner accuracy and inspection scope before field release.

Coverage is essential. A 35% X-coverage scan can achieve P95 = 0.677 mm and ICP
fitness = 0.970 while being shifted about 1.36 m along the repeated extrusion. The
coverage gate correctly rejects that geometrically ambiguous answer when no survey
control is available.

## Candidate support

The 5 mm distance gate is unchanged. The engine now distinguishes:

- `raw_candidate_count`: every point above 5 mm,
- `candidate_count`: points belonging to a spatially supported defect cluster.

A supported cluster currently requires at least 10 points within a 100 mm
neighbourhood. Raw points rejected as sparse remain counted in the prediction. This
prevents one random scanner return from becoming a component-level manufacturing
failure while preserving traceability.

The original detectable translations retain their component identities, candidate
support, point sets, and displacement estimates. The -4 and +4 mm cases now have no
supported cluster, which is the appropriate behaviour below the existing 5 mm gate.

## Controlled robustness matrix

Run directory:

`data/registration_robustness/robust_20261006T121647Z/`

The final source-matched results are under `analysis_final/`. The earlier analysis
directories are retained because the first run exposed the insufficient statistical
outlier filter and motivated the radius and cluster-support correction.

Three local states were used in every scenario: 0, -8, and +12 mm STF_03 translation.

| Scenario | Expected | Result | Mean reference RMSE |
|---|---:|---:|---:|
| Small automatic transform | accept 3 | accepted 3 | 0.1192 mm |
| Opposite-direction automatic transform | accept 3 | accepted 3 | 0.1192 mm |
| Large automatic transform | accept 3 | accepted 3 | 0.1192 mm |
| 70% partial scan + point correspondences | accept 3 | accepted 3 | 0.1603 mm |
| 10% isolated outliers + filter | accept 3 | accepted 3 | 0.1237 mm |
| Arbitrary pose + scanner/survey matrix | accept 3 | accepted 3 | 0.1192 mm |
| 35% partial scan without control | reject 3 | rejected 3 | QA blocked |
| Excessive rotation without control | reject 3 | rejected 3 | QA blocked |

Acceptance behaviour was correct in 24/24 cases. The 18 accepted cases contained
six zero controls and twelve detectable translations:

- zero-control false positives: 0/6,
- detectable cases detected: 12/12,
- correct STF_03 identifications: 12/12,
- valid displacement estimates: 12/12,
- failed executions: 0/24.

For the outlier scenario, each input contained 30,000 structural points plus 3,000
uniform environmental points. The radius filter removed 2,932--2,942 points. In the
zero control, 51 remaining raw points exceeded 5 mm but none formed a supported
cluster; no component or displacement was reported.

The accepted complete-scan scenarios retained approximately +0.375 mm signed local
estimation error. The partial controlled scan produced about +0.038 mm error. That
difference must not be interpreted as an estimator improvement: cropping changes the
sample used by the 2nd/98th percentile envelope and therefore exposes a remaining
coverage sensitivity that needs a dedicated estimator study.

## Reproduction

```powershell
.\.venv\Scripts\python.exe -B src\generate_registration_robustness.py `
  data\validation\y_translation_20261006T104604Z_b8c0a39c `
  <new-run-directory>

.\.venv\Scripts\python.exe -B src\validate_registration_robustness.py `
  <new-run-directory> `
  <new-analysis-directory>
```

Generation and analysis directories are immutable and cannot be overwritten.

## Limits that remain

This is still synthetic validation on one panel topology. Coherent clutter attached
to or very near the structure is not equivalent to isolated outliers and remains a
separate segmentation problem. Survey/control-point quality must be measured in the
actual yard coordinate network. Partial-scan displacement accuracy, varying density,
occlusion by adjacent structure, and multiple scanner stations remain unqualified.
