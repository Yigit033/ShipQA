# ShipQA — Project Context

## Current status update - combined component rigid pose

The controlled combined-pose hardening stage is complete.
ShipQA now derives component center translation and signed XYZ rotation from the
same quality-gated local rigid transform. Unsupported translation axes remain null;
for the current long STF_03 geometry, X remains unobservable while Y and Z have
surface-normal support. When rotation exceeds 0.1 degrees, the engine now publishes
this joint-pose translation instead of the invalid legacy axis-envelope result.

The sequential eleven-case Rhino batch combines Y/Z translations with
single-axis and multi-axis rotations, includes a zero control, and contains two
explicit X-observability characterization cases. Every case is rebuilt from nominal
geometry. The external evaluator completes all predictions before reading ground
truth and reports all denominators.

The preserved historical -8 mm result remains -8.126176 mm. On the newer grouped
-8 mm fixture, the local rigid transform estimates Y as -7.986415 mm with only
0.006572 degrees of residual rotation. Existing pure-rotation STF_03 fixtures keep
inferred Y/Z center translation below 0.015 mm. The completed sweep detected and
correctly identified STF_03 in 10/10 non-zero cases, returned 10/10 valid Y/Z plus
rotation poses, and produced no false positive in the zero control. Y/Z MAE are
0.010382/0.009116 mm; mean/maximum total angular errors are 0.012478/0.020191
degrees. Longitudinal X remains correctly unavailable in 0/2 characterization
cases. All 62 tests pass. See
[COMBINED_COMPONENT_POSE.md](COMBINED_COMPONENT_POSE.md).

## Current status update - local component rotation implementation

The next hardening implementation is ready for Rhino validation. A sequential
13-case batch rotates STF_03 independently around X, Y and Z about its nominal
center. Small RX cases characterize the unchanged 5 mm detection gate; the other
cases test detection, component identity and local rigid rotation recovery.

The external engine now estimates a component-specific nominal-to-observed rigid
transform with robust point-to-plane ICP against grouped nominal component faces.
It reports the rotation matrix, signed XYZ Euler angles, center translation,
fitness, RMSE and source hash without receiving ground truth or the expected axis.

The Rhino rotation sweep completed on 2026-10-07. All 10 detectable rotations were
detected, identified as STF_03 and passed the local pose quality gate. Mean/maximum
total angular error are 0.00935/0.01542 degrees. RX, RY and RZ target-axis MAE are
0.00642, 0.000225 and 0.000336 degrees respectively. The two RX +/-1.5 degree
threshold-characterization cases correctly produced no supported detection, and
the zero control had no false positive.

The first pass revealed an overly narrow coarse robust loss and isotropic
clustering that rejected distributed roll evidence. Progressive ICP robust scales
and a manifest-aligned longest-axis fallback corrected both causes. The unchanged
5 mm gate and 10-point support remain in force. A rerun of the earlier 24-case
registration/outlier matrix retained 24/24 correct acceptance decisions and zero
false positives in all six accepted controls. All 55 tests pass. See
[COMPONENT_ROTATION.md](COMPONENT_ROTATION.md).

Axis-envelope translation assumes an unrotated component. When qualified local
rotation exceeds 0.1 degrees, ShipQA now nulls the legacy translation result and
marks it `invalid_under_component_rotation`, preventing rotation from being
reported as a manufacturing translation before the joint-pose phase.

## Current status update - multi-axis translation hardening

The first implementation step for independent component X/Y/Z translation is
ready for Rhino validation. New nominal OBJ exports preserve PART_ID identity with
OBJ group records, allowing the external engine to isolate the detected
component's nominal faces without receiving an expected axis or ground truth.

The validated Y estimator and all existing detection settings remain unchanged.
Historical ungrouped fixtures continue to produce their original Y result and
explicitly mark XYZ pose as unavailable. A new sequential Rhino batch creates one
zero control plus -16, -8, +8 and +16 mm cases independently on each axis. The
external evaluator predicts every case before reading any ground truth and reports
axis errors, vector errors and complete failure denominators.

The implementation was prepared without changing the preserved STF_03 -8 mm Y
result of -8.126176 mm. See
[MULTI_AXIS_TRANSLATION.md](MULTI_AXIS_TRANSLATION.md).

The Rhino sweep subsequently completed on 2026-10-07. It proved that longitudinal
X motion of the 4000 mm prismatic stiffener is not observable with the current
surface sampling: only 2--8 endpoint points crossed the 5 mm gate, while at least
10 supported points are required. The engine now returns a null X value with
`insufficient_geometric_support` instead of a confident false displacement.

All four Y and all four Z cases were detected and correctly identified as STF_03.
Y bias/MAE/max error are 0.382362/0.382362/0.382362 mm. Z
bias/MAE/max error are 0.371843/0.371843/0.381892 mm. The zero control remains free
of false positives and there were no execution failures. The earlier negative-Z
error was caused by fixed low-Z cropping; Z now uses the observable upper component
surface. Full-vector metrics remain null whenever one axis lacks geometric support.
All 49 tests pass and the preserved -8.126176 mm historical regression is unchanged.

## Current status update - registration robustness and QA gating

The next controlled registration milestone is complete. A focused matrix exercised
eight conditions across zero, -8 mm, and +12 mm local STF_03 states (24 total cases):
small/opposite/large automatic transforms, a 70% partial scan with point controls,
10% isolated environmental points, an arbitrary pose with scanner/survey transform,
a 35% partial scan without control, and excessive rotation without control.

All 18 cases expected to be usable passed registration. All 6 geometrically
untrustworthy cases were rejected before component detection or displacement
estimation. Among accepted cases, zero-control false positives were 0/6; all 12/12
detectable cases identified STF_03 and returned finite estimates. There were no
execution failures. The original six detectable Y-sweep results and preserved
-8.126176 mm regression remain unchanged.

The engine now supports coarse scan-to-CAD input as either a proper 4x4 scanner/
survey transform or at least three non-collinear scan/CAD corresponding points.
Registration quality gates use fine fitness, exact post-registration surface P95,
retained-point fraction, and geometric coverage. A failure returns
`registration_rejected` and does not run component QA.

Optional radius support filters isolated scan points. The unchanged >5 mm gate is
now followed by a minimum spatial-support check: raw threshold points are reported,
while only clusters of at least 10 points within 100 mm can create an engineering
component finding. This removed false positives from residual environmental points
and changed the +/-4 mm characterization cases from sparse identifications to the
more defensible no-supported-detection outcome.

See [REGISTRATION_ROBUSTNESS.md](REGISTRATION_ROBUSTNESS.md) for thresholds,
scenario results, reproduction commands, field-input formats, and remaining limits.

## Current status update - controlled registration milestone

ShipQA now has an optional global rigid scan-to-CAD registration stage in the
external Python engine. It uses deterministic nominal-surface sampling, centroid
translation under a rough-axis prior, and three robust point-to-plane ICP stages.
The estimated scan-to-CAD matrix, stage fitness/RMSE values, exact surface metrics
before and after registration, transform integrity, and provisional quality checks
are written into each structured prediction. The registered point cloud is retained.

A controlled nine-case run applied rotations of (+1.2, -0.8, +1.5) degrees about
the panel center and then (+75, -45, +30) mm translation to every existing scan.
Registration completed 9/9. Mean reference-vertex round-trip RMSE was 0.128933 mm,
maximum reference error was 0.166715 mm, and maximum rotation error was 0.000210
degrees. Pre-registration surface P95 values of 65.249--67.099 mm fell to
0.678--0.993 mm.

Downstream QA still found STF_03 in 6/6 detectable cases, produced 6/6 estimates,
and kept the zero control free of false positives. Detectable-case bias/MAE/max
error became 0.374313/0.374313/0.375206 mm. The registered estimates differ from
the aligned baselines by only 0.0072--0.0100 mm. The existing 5 mm gate, distance
kernel, component identification, and percentile estimator were not changed. The
preserved unregistered -8 mm regression remains exactly -8.126176 mm.

This is a controlled fine-registration result, not a claim of arbitrary-pose or
real-yard readiness. The current coarse initialization requires roughly parallel
scan/CAD axes and complete dominant reference geometry. Partial scans, occlusion,
outliers, repeated-geometry ambiguity, alternative transform magnitudes, and field
coarse-alignment inputs remain to be validated. See
[REGISTRATION_VALIDATION.md](REGISTRATION_VALIDATION.md) for method, results,
reproduction, ground-truth isolation, and limitations.

## Current status update - 2026-10-06

The nine-case Rhino sweep is complete in
`data/validation/y_translation_20261006T104604Z_b8c0a39c`. All six detectable
translations identified STF_03, with constant +0.382362 mm signed error for this
sampling realization. The zero control had no false positive. Both +/-4 mm cases
triggered identification with very sparse candidate support.

The surface-distance discrepancy was traced to float32 barycentric cancellation
inside Open3D 0.20.0 RaycastingScene on long, thin triangles. Raw OBJ indexing,
geometry, import and tensor conversion were verified. A one-triangle reproduction
and independent exhaustive 60-digit triangle distances confirm the root cause.

The engine now uses a float64 face/edge/vertex triangle-distance kernel. No scan,
threshold, units, sampling settings, component identification or percentile shift
estimator changed. All 32 tests pass. All 300,000 scan points (sweep plus preserved
regression) agree with independent analytic surfaces to within 1.14e-13 mm.

Reruns are preserved under `data/distance_investigation/float32_triangle_20261006/`.
The -8 mm regression remains -8.126176 mm. All sweep displacement estimates and
component identities are unchanged. The +4 mm candidate count corrects from 3 to
1; surface statistics are updated. Detectable-case bias/MAE/max error remain
+0.382362/0.382362/0.382362 mm using all 6/6 cases. No rerun failed. Original inputs
and before-results remain unchanged.

See [SURFACE_DISTANCE_INVESTIGATION.md](SURFACE_DISTANCE_INVESTIGATION.md) for
mathematical evidence, reproducible diagnostics, comparisons and trust limits.
Sparse-candidate detection and multi-seed estimation characterization remain open.
Registration has not been implemented. Earlier status sections below are historical.


## Current status update ? 2026-10-05

The -8 mm Rhino experiment is complete: the unchanged detector identified STF_03
and estimated -8.126176 mm (observed center Y = 1191.873824 mm), with absolute error
0.126176 mm and relative absolute error 1.5772%. There were 627 defect candidates
and 1,945 estimation points; surface P95/P99 were 0.785778/7.959970 mm.
The original +12 mm experiment remains the historical +11.874 mm result.

A minimal sequential Rhino Y-translation batch generator and external validation
harness are now implemented. The actual -8 mm dataset, original code and results
are preserved in `data/regression/stf03_negative_8/`; the callable engine reproduces
-8.126176 mm. Detector/estimator mathematics, the 5 mm gate, 30,000 points, sigma
0.35 mm and seed 42 remain unchanged. No bias correction has been applied.

The next action is to run `rhino_scripts/06_generate_y_validation_batch.py` inside
Rhino 7 on a copy of the nominal panel document. The nine-case scans have NOT yet
been generated or validated. Zero (control), +/-4 mm (threshold characterization)
and the six detectable translations have separate scoring policies. All prediction
calls finish before the external evaluator reads ground truth.

See [VALIDATION_HARNESS.md](VALIDATION_HARNESS.md) for exact execution steps,
provenance, scoring, tests and limitations. The numbered sections below retain the
original experiment history; statements that the -8 mm test is pending are historical
and are superseded by this update.


## 1. Project Summary

ShipQA is an engineering software project for dimensional quality assurance of ship structures.

The long-term objective is to compare a nominal ship structure CAD model against an as-built 3D scan / point cloud and automatically determine:

- where manufacturing deviations exist
- which engineering component is affected
- how much the component has shifted, rotated, deformed, or deviated
- whether the deviation is inside or outside engineering tolerance
- how the result should be visualized and reported to an engineer

The project combines:

- ship and marine engineering knowledge
- CAD automation
- Rhino
- Python
- computational geometry
- point-cloud processing
- numerical QA
- and later, where technically justified, AI / machine learning

The product is not intended to be merely a Rhino script collection.

The intended end result is a practical engineering QA system that could eventually be used in a shipyard, design office, manufacturing inspection workflow, repair/refit environment, or related heavy-industry setting.

---

# 2. Product Problem

In a real shipbuilding or heavy-industry workflow, there is normally a difference between:

1. what was designed
2. what was actually manufactured

The design side contains the nominal/reference geometry.

The physical structure can be measured using technologies such as laser scanning or other 3D measurement systems.

The resulting measured geometry can be represented as a point cloud.

ShipQA aims to compare those two states:

```text
NOMINAL DESIGN
"What should exist"

        versus

AS-BUILT SCAN
"What was actually manufactured"

The system should then convert raw geometric differences into useful engineering information.
A final system should not merely say:
There are points with 8.4 mm deviation.

It should ideally say something closer to:
Component: STF_03
Status: FAIL

Nominal transverse position:
1200.000 mm

Observed transverse position:
1192.180 mm

Estimated transverse displacement:
-7.820 mm

Tolerance:
±3.000 mm

That difference is central to the product.
ShipQA should become an engineering inspection tool rather than only a visualization tool.
3. Important Terminology
Nominal Geometry
The reference/design geometry.
It represents where the structure is supposed to be according to the design.
In the current prototype, the nominal geometry is generated inside Rhino.
As-Built Geometry
A representation of the manufactured structure.
For the current synthetic prototype, this geometry is deliberately generated from the nominal model and modified with known defects.
This allows the system to be tested against exact ground truth.
In a future real-world workflow, ShipQA would not normally create the as-built CAD geometry itself.
The actual structure would be measured through scanning.
Point Cloud / Scan
A point cloud is a collection of 3D XYZ measurement points representing physical surfaces.
Each point contains spatial coordinates.
For example:
X = 1834.522 mm
Y = 1192.071 mm
Z = 147.281 mm

The current project does not yet use a real laser scanner.
Instead, the Rhino-side prototype creates a synthetic point cloud from intentionally modified as-built geometry.
This lets the geometric analysis pipeline be developed and validated before obtaining real shipyard scan data.
Surface Deviation
The shortest distance between a measured scan point and the nominal CAD surface.
This is useful for detecting geometric differences.
However, surface deviation is NOT automatically the same thing as component displacement.
For example:
If a T-stiffener is translated 12 mm in Y, not every point belonging to that stiffener will necessarily have a 12 mm nearest-surface distance.
Some surfaces can overlap or remain close to other nominal surfaces.
This distinction has already been observed in the current prototype.
Component Pose Deviation
The displacement and eventually rotation of an engineering component relative to its nominal pose.
Examples include:
STF_03 shifted +12 mm in Y

Bracket B_14 rotated 1.7 degrees

Plate P_08 displaced +5.3 mm in Z

This is more valuable than raw point-to-surface distance because it describes what happened to an actual engineering component.
4. Current Technology Stack
Rhino Side
Current version:
Rhino 7

Rhino 8 is NOT currently being used.
Rhino 7 uses:
IronPython 2.7
RhinoCommon
rhinoscriptsyntax

Rhino is currently responsible for:
- generating nominal CAD geometry
- generating controlled as-built test geometry
- storing engineering component metadata
- generating synthetic point-cloud data
- exporting nominal mesh geometry
- exporting engineering component metadata
- later, importing and visualizing QA results
Rhino should eventually function primarily as the CAD interaction and engineering visualization layer.
External Python Engine
Location:
src/

Current Python version:
Python 3.12.6

Virtual environment:
.venv

Current confirmed packages:
Open3D 0.20.0
NumPy 2.5.3
pip 26.1.1

The external engine is intentionally separate from Rhino 7.
It will handle computationally heavier and modern Python tasks such as:
- point-cloud processing
- geometric analysis
- registration
- deviation calculations
- component localization
- component pose estimation
- engineering tolerance evaluation
- future reporting
- future FastAPI integration
- future ML / AI functionality
5. Current Repository Location
The local project is stored at:
C:\active_projects\ShipQA

VS Code is used as the primary code editor.
The Codex VS Code extension may also be used to work directly on this repository.
Rhino's built-in Python editor should NOT be treated as the main development environment anymore.
Rhino scripts should be edited in VS Code and executed from Rhino using:
RunPythonScript

6. Current Repository Structure
The intended structure is:
ShipQA/
│
├── AGENTS.md
│
├── docs/
│   └── PROJECT_CONTEXT.md
│
├── rhino_scripts/
│   ├── 01_build_nominal_panel.py
│   ├── 02_build_asbuilt_test.py
│   ├── 03_generate_synthetic_scan.py
│   ├── 04_export_nominal_mesh.py
│   └── 05_export_component_manifest.py
│
├── src/
│   └── qa_engine.py
│
├── data/
│   ├── nominal_reference.obj
│   ├── synthetic_scan_v1.xyz
│   ├── component_manifest.json
│   └── defect_candidates.xyz
│
├── tests/
│
└── .venv/

Some auxiliary files such as:
requirements.txt
.gitignore
README.md

may be added later.
7. Synthetic Nominal Test Structure
The current first prototype is intentionally simple.
It consists of a stiffened plate representing a simplified ship structural panel.
The model contains:
Main plate
+
5 longitudinal T-stiffeners

The nominal model dimensions currently used are:
Panel length:
4000 mm

Panel width:
2400 mm

Plate thickness:
12 mm

Stiffener web height:
150 mm

Stiffener web thickness:
8 mm

Stiffener flange width:
80 mm

Stiffener flange thickness:
10 mm

The five nominal stiffener center positions are:
STF_01 → Y = 400 mm
STF_02 → Y = 800 mm
STF_03 → Y = 1200 mm
STF_04 → Y = 1600 mm
STF_05 → Y = 2000 mm

8. Engineering Metadata
The Rhino model contains engineering metadata.
Examples include:
PART_ID
ASSEMBLY_ID
MODEL_TYPE
KNOWN_DEFECT
KNOWN_SHIFT_Y_MM

Typical identifiers include:
STF_03
STF_03_WEB
STF_03_FLANGE

The important distinction is:
ASSEMBLY_ID = engineering component identity

PART_ID = individual geometric part identity

For example, a T-stiffener contains both a web and flange, but both belong to the same assembly:
ASSEMBLY_ID = STF_03

Component identity must not depend on Rhino object order.
9. Rhino Script Responsibilities
01_build_nominal_panel.py
Purpose:
Generate the clean nominal/reference panel.
It creates:
1 main plate
5 T-stiffeners

It also assigns engineering identifiers and layers.
Current nominal layers include:
00_NOMINAL_PLATE
01_NOMINAL_STIFFENERS

This script defines what the structure is supposed to look like.
02_build_asbuilt_test.py
Purpose:
Create an intentionally defective as-built model from the nominal model.
The script copies nominal geometry to:
05_ASBUILT_GEOMETRY

and then applies a controlled defect.
The current test component is:
STF_03

Initially, the controlled test used:
STF_03 = +12.0 mm Y translation

This test was successfully analyzed.
The current new validation case has been changed to:
STF_03 = -8.0 mm Y translation

Before recreating the as-built model, the script now removes old geometry from:
05_ASBUILT_GEOMETRY

to prevent duplicate test geometry from accumulating.
The controlled defect is stored as metadata.
The current intended metadata includes:
KNOWN_DEFECT
KNOWN_SHIFT_Y_MM

This information is ground truth only.
The QA engine MUST NOT use this value to generate its prediction.
03_generate_synthetic_scan.py
Purpose:
Generate a synthetic point cloud from the as-built geometry.
The current synthetic scan contains:
30,000 points

The points are sampled from the surface geometry.
Gaussian measurement noise is added.
Current noise setting:
sigma = 0.35 mm

This approximates a simplified scanner measurement process.
The script outputs:
synthetic_scan_v1.xyz

The scan therefore represents the intentionally defective as-built structure.
This means the current conceptual pipeline is:
Nominal CAD
    ↓

Create known manufacturing defect
    ↓

As-built synthetic geometry
    ↓

Sample geometry
    ↓

Synthetic scan / point cloud

The script previously contained hardcoded metadata saying:
STF_03 +12 mm Y SHIFT

That is now being corrected so that the scan metadata is read dynamically from:
KNOWN_SHIFT_Y_MM

stored in the as-built geometry.
The modifications have already been made to the script.
The updated -8 mm scan has NOT yet been generated and analyzed.
That is the immediate next development step.
04_export_nominal_mesh.py
Purpose:
Export the nominal Rhino geometry as a triangle mesh that the external Python QA engine can analyze.
Current output:
nominal_reference.obj

The nominal CAD geometry is converted into triangles.
Current exported mesh statistics:
Vertices:
88

Triangles:
132

Units are millimetres.
The external Python engine uses this mesh as the geometric reference.
05_export_component_manifest.py
Purpose:
Export engineering component identities and nominal spatial information from Rhino.
Current output:
component_manifest.json

The manifest contains information such as:
assembly_id
part_ids
bounding box
nominal center

The current exported stiffener centers are:
STF_01 → center Y = 400.000 mm
STF_02 → center Y = 800.000 mm
STF_03 → center Y = 1200.000 mm
STF_04 → center Y = 1600.000 mm
STF_05 → center Y = 2000.000 mm

This manifest allows the external analysis engine to translate geometric anomaly locations into engineering component identities.
10. qa_engine.py
The external analysis engine currently lives at:
src/qa_engine.py

It currently performs several stages.
Stage A — Data Validation
The engine loads:
data/synthetic_scan_v1.xyz
data/nominal_reference.obj
data/component_manifest.json

The latest confirmed validation output for the +12 mm experiment was:
Scan points:
30,000

Mesh vertices:
88

Mesh triangles:
132

Scan bounds:
Min XYZ:
[-0.7995, -0.5856, -1.3038]

Max XYZ:
[4000.5602, 2401.0769, 173.1391]

Nominal CAD bounds:
Min XYZ:
[0, 0, 0]

Max XYZ:
[4000, 2400, 172]

These values were considered reasonable.
The slight scan expansion outside nominal bounds was expected due to the synthetic scanner noise.
11. Surface Deviation Analysis
Open3D's tensor geometry / RaycastingScene functionality is currently used to determine the nearest distance between every scan point and the nominal mesh.
Conceptually:
scan point
    ↓
nearest nominal triangle surface
    ↓
distance in millimetres

For the controlled +12 mm STF_03 displacement experiment, the measured surface-deviation statistics were:
Minimum deviation:
0.000 mm

Median deviation:
0.250 mm

Mean deviation:
0.612 mm

P95:
1.052 mm

P99:
11.962 mm

Maximum:
13.430 mm

Threshold counts were:
> 1 mm:
1,547 points
5.16%

> 2 mm:
1,347 points
4.49%

> 5 mm:
670 points
2.23%

> 10 mm:
582 points
1.94%

Interpretation:
Most of the structure remains close to the nominal geometry.
A relatively small region contains large deviations.
The P99 value of:
11.962 mm

strongly corresponded with the intentionally injected:
+12.0 mm

defect.
However, this statistic alone is not sufficient to conclude that the component itself moved exactly 11.962 mm.
Surface deviation and component displacement remain conceptually separate.
12. Defect Localization
Candidate defect points are currently defined using:
surface deviation > 5 mm

For the +12 mm STF_03 experiment:
Candidate points:
670

The detected defect-region bounds were:
Min XYZ:
[0.1420, 1209.0014, 17.0821]

Max XYZ:
[3985.3402, 1253.0153, 172.7651]

The calculated defect-region center was:
X = 1967.631 mm
Y = 1220.790 mm
Z = 99.030 mm

The worst detected point was:
XYZ:
[2332.8374, 1217.4304, 135.9316]

Deviation:
13.430 mm

Candidate points are exported to:
data/defect_candidates.xyz

13. Component Identification
The QA engine reads:
component_manifest.json

and compares the detected anomaly location with nominal component locations.
For the +12 mm experiment, the result was:
Defect center Y:
1220.790 mm

Detected component:
STF_03

Nominal center Y:
1200.000 mm

Distance to nominal:
20.790 mm

The important successful result is:
Detected component = STF_03

The:
20.790 mm

value must NOT be interpreted as the stiffener displacement.
It is merely the distance between the mean Y-coordinate of selected high-deviation points and the nominal component center.
That metric is biased because only a subset of the component surfaces exceeds the chosen deviation threshold.
This distinction was explicitly identified during development.
14. Component Shift Estimation
A separate component-level calculation was therefore implemented.
Instead of using only high-deviation points, the QA engine selects scan points from the geometric region surrounding the identified component.
For STF_03, this resulted in:
Points used:
1,945

A robust Y-envelope was estimated using percentiles rather than raw min/max values.
Latest successful +12 mm test result:
Component:
STF_03

Nominal center Y:
1200.000 mm

Observed Y envelope:
1172.388 mm
to
1251.359 mm

Observed center Y:
1211.874 mm

Estimated Y shift:
+11.874 mm

Known injected shift:
+12.000 mm

Estimated shift:
+11.874 mm

Absolute error:
0.126 mm

Approximate relative error:
1.05%

This is the strongest successful result produced by the project so far.
However, it represents only one clean synthetic experiment.
It does NOT prove that the approach is robust to arbitrary ship structures or real scanner data.
15. Why a Second Test Is Being Performed
A single successful +12 mm test is not enough.
The current next validation case deliberately changes both:
direction
and
magnitude

of the defect.
The current controlled test is:
Component:
STF_03

Translation:
-8.0 mm in Y

Expected nominal center:
1200 mm

Expected as-built center:
approximately 1192 mm

The QA engine should independently recover a result close to:
-8 mm

without reading the ground-truth metadata.
This test checks whether the system genuinely estimates geometry rather than accidentally depending on the original +12 mm scenario.
16. Current Exact Development State
The project is currently paused at the following point:
Completed
Nominal synthetic panel created.
Five T-stiffeners created.
Engineering metadata assigned.
Controlled +12 mm STF_03 as-built geometry created.
30,000-point synthetic scan created.
Nominal OBJ mesh exported.
Component manifest exported.
External Python 3.12 environment created.
Open3D installed.
NumPy installed.
Nominal mesh and scan loaded successfully.
Point-to-mesh surface deviation implemented.
Defect candidate localization implemented.
Engineering component identification implemented.
STF_03 successfully identified.
Component transverse shift estimator implemented.
Known +12 mm test recovered as:
+11.874 mm

Old as-built geometry is now deleted before rebuilding a new test.
The controlled test shift has been changed from:
+12 mm

to:
-8 mm

The synthetic-scan script has been modified so the ground-truth metadata is no longer hardcoded as +12 mm.
Not Yet Completed
The new -8 mm synthetic scan has NOT yet been generated.
The QA engine has NOT yet been run against the -8 mm dataset.
The -8 mm estimation accuracy is therefore unknown.
That is the immediate next milestone.
17. Immediate Next Milestone
The next action should be:
Run 03_generate_synthetic_scan.py
against the current -8 mm as-built geometry.

This should overwrite/regenerate:
data/synthetic_scan_v1.xyz

with scan data representing the new:
STF_03 -8 mm Y shift

case.
Then:
python src/qa_engine.py

should be run.
The resulting outputs should be checked for:
surface-deviation distribution
defect localization
component identification
component shift estimation

The target result is:
Detected component:
STF_03

Estimated Y shift:
approximately -8 mm

The exact error should be recorded.
18. Validation Philosophy
Development should proceed by increasingly difficult ground-truth experiments.
The system should NOT immediately jump to real ship blocks.
A recommended progression is:
Test 1
single +Y translation

Test 2
single -Y translation

Test 3
smaller translation

Test 4
larger translation

Test 5
different stiffener

Test 6
different scanner noise levels

Test 7
rotation

Test 8
multiple simultaneous component defects

Test 9
missing component

Test 10
plate deformation

Test 11
partial scan / occlusion

Test 12
outlier contamination

Test 13
unknown global scan offset

Test 14
CAD↔scan registration

Test 15
real point-cloud dataset

Each test should have known ground truth when possible.
Accuracy should be measured rather than visually judged.
19. Important Technical Limitation — Registration
The current synthetic scan and nominal model already exist in approximately the same coordinate system.
That is NOT realistic enough for the final product.
Real scanner data may be:
- translated
- rotated
- partially visible
- noisy
- occluded
- contaminated with unrelated geometry
Therefore CAD-to-scan registration will eventually become one of the most important technical parts of ShipQA.
Future registration architecture should likely distinguish:
coarse/global registration
        ↓
fine registration
        ↓
component/local registration
        ↓
manufacturing deviation estimation

A central engineering requirement is:
registration error must not be mistaken for manufacturing error

This is one of the major technical risks of the project.
20. Future Defect Types
The current prototype handles only controlled transverse translation of one stiffener.
Future system versions should investigate:
X displacement
Y displacement
Z displacement

rotation

local deformation

plate buckling / warping

missing stiffener

missing bracket

incorrect component position

incorrect component orientation

possibly dimensional/profile mismatch

Multiple simultaneous defects should eventually be supported.
21. Future Point-Cloud Realism
The current synthetic scan is deliberately simplified.
It currently includes:
surface sampling
Gaussian coordinate noise

It does NOT yet properly simulate:
scanner viewpoint
line of sight
occlusion
shadow regions
varying density
reflective steel surfaces
outliers
scanner range effects
multi-scan registration
partial coverage
environmental clutter

These should be added progressively only after the basic geometric QA pipeline remains stable.
22. AI / Machine Learning Strategy
Artificial intelligence is NOT currently required for the core millimetre-level distance calculation.
Deterministic geometry should remain responsible for measurements whenever possible.
AI should later be evaluated for problems where deterministic approaches become insufficient.
Possible future AI applications include:
point-cloud semantic segmentation

automatic detection of:
- plate
- stiffener
- bracket
- pipe
- equipment

instance segmentation

automatic scan↔CAD component correspondence

anomaly classification

missing-component recognition

complex deformation classification

A later engineering assistant may also use LLM/RAG technology for:
inspection explanation
QA report assistance
engineering-document retrieval
class/tolerance knowledge
natural-language querying

However:
LLMs should not be trusted to perform dimensional metrology.

Numerical QA should remain deterministic and traceable.
23. Future Rhino Integration
The current workflow requires manually running multiple scripts.
That is intentional during early algorithm development.
The eventual product should NOT require users to understand these scripts.
The final Rhino experience may evolve toward:
ShipQA toolbar / plugin

or commands such as:
ShipQA_LoadReference
ShipQA_LoadScan
ShipQA_Register
ShipQA_RunInspection
ShipQA_ShowDeviations
ShipQA_GenerateReport

A user should eventually interact through a controlled UI rather than manually editing Python files.
Rhino may display:
deviation heatmaps

failed components

pass/fail states

selected-component QA information

location markers

engineering annotations

24. Future External Service Architecture
The external Python engine may later become a local service.
A possible architecture is:
Rhino
  ↓
local API
  ↓
FastAPI
  ↓
ShipQA analysis engine
  ↓
Open3D / NumPy / ML
  ↓
result JSON
  ↓
Rhino visualization

The service should preferably support local/on-premise operation.
This is important because shipyard, naval, industrial, and defense-related geometry may be sensitive.
Cloud dependence should not be assumed.
25. Future Reporting
The final system should produce engineering-readable QA results.
Possible report content:
Project ID
Block / assembly ID
scan date
reference revision
scan file
registration quality

component ID
nominal location
measured location
translation
rotation
surface deviation
tolerance
pass/fail status

worst deviations
visual screenshots
heatmaps
inspection notes

Reports should be traceable and reproducible.
26. Potential Product Workflow
A mature ShipQA workflow could eventually resemble:
Engineer selects project/block
        ↓
Nominal model is loaded
        ↓
As-built scan is imported
        ↓
Scan preprocessing
        ↓
Global registration
        ↓
Fine registration
        ↓
Deviation calculation
        ↓
Component identification
        ↓
Component pose/deformation estimation
        ↓
Tolerance evaluation
        ↓
Failed components highlighted in Rhino
        ↓
QA report generated

Long term, reference geometry may be obtained directly from a shipyard CAD/PDM/PLM workflow rather than manually uploaded.
27. Product Differentiation
The goal is NOT to differentiate the developer simply by writing Rhino Python scripts.
Modern engineers can increasingly generate simple CAD automation scripts using AI tools.
The intended differentiation is deeper:
understanding the ship structure
+
understanding the CAD model
+
building deterministic geometric analysis
+
processing 3D scans
+
mapping anomalies back to engineering components
+
building a repeatable QA workflow
+
later adding AI where it produces real value

The product should automate an engineering process rather than merely automate a Rhino command.
28. Current Product Hypothesis
The core hypothesis being tested is:
Can we take nominal ship-structure geometry
and an as-built point cloud,
then automatically determine
which component is wrong
and quantify the physical deviation
with engineering-useful accuracy?

The current +12 mm experiment provides an encouraging first result.
It does not yet validate the full product.
The project should continue treating this as an engineering hypothesis that must be tested under progressively harder conditions.
29. Known Risks
Major risks include:
Registration risk
Incorrect global alignment can create false manufacturing defects.
Geometry ambiguity
Nearest-surface distances do not uniquely identify rigid component motion.
Partial scans
Real structures may not be fully observable.
Occlusion
Other components can hide parts of the structure.
Steel scanning conditions
Reflective surfaces and environmental effects may degrade scan quality.
Component correspondence
Determining which measured geometry corresponds to which CAD component may become difficult in dense ship structures.
Scale
A complete ship block can contain far more geometry and points than the current small prototype.
Tolerance context
Different components and production stages may require different allowable tolerances.
Data access
Real shipyard CAD and scan datasets may be difficult to obtain.
These risks should be addressed progressively rather than hidden behind AI.
30. Engineering Integrity Rules
The following principles must remain true throughout development:
Never use ground truth to generate the prediction.

Never call a nearest-surface distance a component translation without justification.

Never assume registration is perfect for real scans.

Never add AI simply to make the product sound advanced.

Never judge accuracy only from a visualization.

Never change units silently.

Never rely on object order as engineering identity.

Never claim a feature is validated when it has only been demonstrated once.

31. Current Ground-Truth Experiment Summary
Experiment 1
Component:
STF_03

Nominal center:
Y = 1200.000 mm

Injected translation:
+12.000 mm Y

Synthetic scan:
30,000 points

Scanner noise:
sigma = 0.35 mm

Detected component:
STF_03

Estimated center:
Y = 1211.874 mm

Estimated translation:
+11.874 mm

Absolute translation error:
0.126 mm

This test passed as an initial proof of concept.
Experiment 2 — Current
Component:
STF_03

Nominal center:
Y = 1200.000 mm

Injected translation:
-8.000 mm Y

Expected as-built center:
approximately 1192.000 mm

Status:
As-built geometry generated.

Synthetic scan script updated.

New -8 mm synthetic scan not yet generated.

QA analysis not yet performed.

This is the immediate active experiment.
32. Definition of Success for the Current Prototype Phase
Before moving seriously toward AI, UI, SaaS, or a Rhino plugin, the prototype should demonstrate that:
1. Known geometric defects can be injected reproducibly.

2. Synthetic scan generation is reproducible.

3. Surface deviations can be calculated correctly.

4. Defect regions can be localized.

5. Engineering components can be identified.

6. Component displacement can be estimated independently.

7. Both positive and negative translations can be recovered.

8. The method survives realistic amounts of measurement noise.

9. Registration errors can eventually be separated from manufacturing defects.

10. Results can be measured against known ground truth.

Only after this foundation is credible should the project substantially increase complexity.
33. Near-Term Development Roadmap
The immediate sequence should be:
CURRENT
↓
Validate STF_03 -8 mm case
↓
Record numerical error
↓
Automate repeated synthetic validation cases
↓
Test several displacement magnitudes
↓
Test another stiffener
↓
Test multiple noise levels
↓
Turn experiments into repeatable tests
↓
Add controlled rotation
↓
Improve component pose estimation
↓
Introduce global CAD↔scan misalignment
↓
Implement registration
↓
Test partial / noisy / occluded scans
↓
Improve Rhino result visualization
↓
Evaluate real point-cloud data
↓
Only then evaluate AI component recognition

34. Long-Term Product Vision
A mature version of ShipQA should allow an engineer to provide or connect:
nominal production geometry
+
as-built 3D scan

and receive:
automatic registration

deviation heatmap

component-level defect identification

translation / rotation / deformation estimates

engineering tolerance evaluation

PASS / FAIL classification

Rhino visualization

traceable QA report

A user should not need to manually run the current prototype scripts.
The current scripts exist to prove and understand the engineering core before building a polished product around it.
35. Guidance for Codex
Before continuing development:
1. Read AGENTS.md.
2. Read this entire document.
3. Inspect the current repository.
4. Do not assume that documentation is more current than actual code if they conflict.
5. Preserve the successful +12 mm validation logic.
6. Do not use ground-truth defect metadata as input to the prediction algorithm.
7. Keep Rhino 7 / IronPython 2.7 compatibility in rhino_scripts.
8. Keep modern numerical processing in the Python 3.12 external engine.
9. Make small, testable changes.
10. Report assumptions and failures explicitly.
The immediate task after reading this context is NOT to redesign the architecture.
The immediate task is to continue the current controlled validation experiment:
STF_03
-8.0 mm Y translation

generate the corresponding synthetic point cloud, run the existing QA engine, measure the recovered displacement, and compare the estimate against the known ground truth.
Do not skip this validation in favor of adding new features.
