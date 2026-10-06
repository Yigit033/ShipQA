# Surface distance investigation - 2026-10-06

## Confirmed root cause

On the installed Open3D 0.20.0 CPU build, `RaycastingScene` loses accuracy in its
float32 point-to-triangle primitive for long, thin triangles. The interior closest
point is calculated using differences of large dot-product products. Cancellation
corrupts barycentric coordinates and can make the scene select a different,
farther triangle. The failure is in primitive arithmetic, not the displacement
estimator or the 5 mm gate.

The relevant implementation is `closestPointTriangle` in the
[Open3D v0.20.0 source](https://github.com/isl-org/Open3D/blob/v0.20.0/cpp/open3d/t/geometry/RaycastingScene.cpp#L190-L253).
Local reproduction, arithmetic intermediates and installed build configuration are
recorded in `data/distance_investigation/float32_triangle_20261006/verified_root_cause.json`.

## Minimal reproduction and mathematical verification

One triangle, in millimetres:

```text
A = (4000, 1240, 172)
B = (   0, 1240, 162)
C = (   0, 1240, 172)
P = (1364.6434, 1240.4315, 168.5501)
```

The perpendicular projection is `(1364.6434, 1240, 168.5501)`. Its barycentric
weights are `(0.34116085, 0.34499, 0.31384915)`: all positive and summing to one.
It is inside the triangle, so the shortest distance is exactly **0.4315 mm**.

With that triangle alone, Open3D returns **3.414161205 mm** and closest point
`(1361.2567138671875, 1240, 168.5706787109375)`.

The float32 interior arithmetic reproduces this wrong point exactly. For example,
`vc = d1*d4 - d3*d2` subtracts products of approximately -5.754e13:

| Arithmetic | va | vb | vc |
|---|---:|---:|---:|
| float32 | 545259520 | 549453824 | 507510784 |
| float64 | 545857360 | 551984000 | 502158640 |

Reordering the same triangle's vertices changes the Open3D result from 3.414161 mm
to approximately 0.431519 mm, despite representing exactly the same surface.
The query's float32 rounding is only 0.0000379113 mm. A 60-digit calculation using
the **same rounded float32 inputs** gives 0.4315185546875 mm. Thus input rounding
cannot explain the millimetre-scale discrepancy; it is internal arithmetic.
The installed scene rejects float64 queries, so changing only the input dtype
is not a fix. The diagnostic intentionally probes and records that rejection.

In the complete mesh, the scene chooses triangle 32 instead of the actual closest
triangle 98 and reports **3.168009281 mm** for the same point. The one-triangle
example removes spatial traversal, other components and overlaps from the cause.

Run the standalone reproduction:

```powershell
.\.venv\Scripts\python.exe -B tests\reproduce_raycasting_error.py
```

## Alternatives checked

| Possible source | Evidence |
|---|---|
| OBJ export | Directly parsed raw OBJ triangles contain the correct face and interior projection. No export changes were needed. |
| Indexing/topology | All 132 faces have valid 1-based indices and positive area. The minimal case uses indices 0,1,2 and still fails. |
| Open3D import | OBJ has 264 vertex records; import merges these into 88 positions. The multiset of triangle coordinates is exactly preserved. |
| Tensor conversion | Integer millimetre vertex coordinates are exactly representable in float32. Triangle-coordinate multisets before/after conversion are identical. |
| Duplicate/overlapping geometry | There are no duplicate geometric triangles or zero-area faces. Structural contacts exist, but one isolated triangle already reproduces the problem. |
| Audit error | Independent exhaustive 60-digit distances against every raw OBJ triangle agree with the analytic box distances at all nine recorded points. |
| Numerical precision | Single-triangle failure, intermediate arithmetic, vertex-order dependence and accurate float64/Decimal results isolate cancellation in the float32 primitive. |

An independent reference implementation in `tests/distance_reference.py` enumerates
vertices, clamped edges and an interior planar stationary point. Its barycentric
solve uses Decimal arithmetic with 60 digits. The production kernel instead uses
float64 cross-product half-space tests; they are separate implementations.

All nine recorded point comparisons, in mm:

| Case | Previous Open3D | Decimal exhaustive OBJ | Corrected kernel |
|---|---:|---:|---:|
| 001 | 17.018175 | 17.013900 | 17.013900 |
| 002 | 13.019485 | 13.013900 | 13.013900 |
| 003 | 9.021954 | 9.013900 | 9.013900 |
| 004 | 5.028353 | 5.013900 | 5.013900 |
| 005 | 3.168009 | 0.431500 | 0.431500 |
| 006 | 5.430325 | 4.431500 | 4.431500 |
| 007 | 9.010257 | 9.010200 | 9.010200 |
| 008 | 13.010257 | 13.010200 | 13.010200 |
| 009 | 17.010256 | 17.010200 | 17.010200 |

## Primitive-level fix

`src/surface_distance.py` computes unsigned distances in float64 to the actual
triangles. For each triangle it checks all three clamped edges (including vertices)
and the perpendicular plane distance when inside the three oriented edge
half-spaces. No subtraction of large dot-product products is needed for the
interior test. Degenerate triangles reduce to segments/points.

The search is exhaustive over all triangles: it does not refine only the triangle
selected by the inaccurate old query. The kernel needs no component identity,
bounding-box oracle or ground truth. Nominal geometry still comes from the OBJ.

Complexity is O(scan points * triangles), with O(scan points) working memory. This
is an intentional correctness-first choice for the 132-triangle prototype, not a
vessel-scale acceleration scheme. No dependency was added or upgraded. Rhino
scripts, scans, units, sampling settings, identification logic, percentile shift
estimator and the strict >5 mm gate were unchanged.

Each prediction now records the distance method and kernel source hash. The
validation runner accepts `--output-dir` for a fresh analysis directory, so existing
scans can be reanalyzed without overwriting earlier results. All predictions still
finish before ground truth is read.

## Before/after validation

Original run: `data/validation/y_translation_20261006T104604Z_b8c0a39c`.
New output: `data/distance_investigation/float32_triangle_20261006/sweep_after`.
The preserved -8 mm regression output is in the adjacent `regression_after` folder.
Source snapshots and original artifact checksums are preserved in the investigation
directory. All original input and result checksums were verified after the reruns.

Surface statistics below are mm. Arrows mean before -> after.

| Case / injected shift | P95 | P99 | Candidates | Predicted Y shift, unchanged |
|---|---|---|---|---:|
| Preserved -8 | 0.785778 -> 0.769910 | 7.959970 -> 7.957308 | 627 -> 627 | -8.126176 |
| Sweep -16 | 1.035226 -> 0.990695 | 15.916157 -> 15.916120 | 1252 -> 1252 | -15.617638 |
| Sweep -12 | 1.005700 -> 0.973400 | 11.961578 -> 11.961630 | 673 -> 673 | -11.617638 |
| Sweep -8 | 0.780501 -> 0.768620 | 7.994145 -> 7.994201 | 633 -> 633 | -7.617638 |
| Sweep -4 | 0.958395 -> 0.929655 | 4.013554 -> 4.013603 | 1 -> 1 | -3.617638 |
| Sweep 0 | 0.686100 -> 0.677805 | 0.929501 -> 0.898803 | 0 -> 0 | None |
| Sweep +4 | 0.956735 -> 0.927345 | 4.021498 -> 4.011816 | 3 -> 1 | +4.382362 |
| Sweep +8 | 0.774805 -> 0.762705 | 7.968814 -> 7.967703 | 609 -> 609 | +8.382362 |
| Sweep +12 | 0.994450 -> 0.968655 | 11.952631 -> 11.947155 | 643 -> 643 | +12.382362 |
| Sweep +16 | 1.024580 -> 0.982235 | 15.907724 -> 15.906403 | 1252 -> 1252 | +16.382362 |

- Every nonzero case still identifies STF_03. Sweep estimates use 1,915 points;
  the preserved regression uses 1,945, unchanged before/after.
- Zero control: no candidates, component or estimate, and 0/1 false positives
  before/after. Its maximum distance corrects from 3.168009 to 1.403000 mm.
- Both +/-4 mm cases now have one genuine >5 mm measured surface-distance
  candidate (maxima 5.0139 and 5.0102 mm). Their estimates remain threshold
  characterization, not six-case accuracy inputs. The previous extra +4 candidates
  came from inaccurate distances; single-candidate identification remains a
  separate detector-support limitation.
- Detectable translations: 6/6 detected, 6/6 correct IDs, 6/6 valid estimates,
  0 excluded, with bias +0.382362 mm, MAE 0.382362 mm, maximum absolute error
  0.382362 mm before and after. This distance fix does not remove sampling bias.
- Preserved regression: same -8.126176 mm estimate and 0.126176 mm absolute error.
- No execution failures in either rerun (0/9 sweep, 0/1 preserved regression).

Full-precision comparison: `before_after.csv` in the investigation directory.
The 5 mm threshold and displacement estimator were not adjusted to obtain this result.

## Tests and trust boundary

All 32 tests pass. Added coverage includes face/edge/vertex minima, thin triangles
under every vertex permutation, degenerate triangles, random meshes against the
60-digit exhaustive oracle, rotation/translation/scale invariance, invalid inputs,
the recorded failures, and separate analysis outputs preserving original data and
ground-truth isolation. The old surface statistics are preserved as historical
evidence rather than being treated as mathematically correct regression targets.
The displacement regression and detector/estimator expression checks remain intact.

All **300,000** points (nine sweep scans plus preserved regression) were also checked
against the independent analytic nominal box surfaces. Maximum disagreement was
**1.1313172620930345e-13 mm**; candidate masks agreed exactly. This is numerical
agreement on the known model, not a statement about scanner accuracy.

To reproduce the full diagnostic, choose a new report filename:

```powershell
.\.venv\Scripts\python.exe -B tests\diagnose_surface_distance.py <new_report.json> --verify-all
.\.venv\Scripts\python.exe -B -W ignore::DeprecationWarning -m unittest discover -s tests -v
```

The corrected distance calculation is defensible for this prototype and the tested
triangle/coordinate regimes. It is not a claim of certification for arbitrary
ill-conditioned meshes, extreme coordinate magnitudes or vessel-scale performance.
Input validation rejects nonfinite geometry and out-of-range indices, and arithmetic
overflow is explicit. Sparse-candidate detection support and multi-seed estimation
characterization remain separate open work. Registration has not been implemented.
