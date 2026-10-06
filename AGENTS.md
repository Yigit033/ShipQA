# AGENTS.md

## Project: ShipQA

ShipQA is an engineering software project for dimensional quality assurance of ship structures.

The long-term objective is to compare nominal ship CAD geometry against as-built 3D scan / point-cloud data and automatically identify, quantify, visualize, and report manufacturing deviations at component level.

This is not primarily a demo, chatbot, or generic AI project. Engineering correctness, traceability, numerical reliability, and practical shipyard usefulness take priority over adding AI features.

---

## 1. Core Engineering Principle

Always distinguish between:

1. Nominal geometry:
   The intended/design geometry from CAD.

2. As-built geometry:
   The geometry that represents what was actually manufactured.

3. Scan / point cloud:
   A measured or simulated representation of the as-built structure.

4. Surface deviation:
   Distance from measured scan points to nominal CAD surfaces.

5. Component pose deviation:
   Translation/rotation/deformation of an engineering component relative to its nominal position.

These concepts must not be conflated.

For example:

A stiffener shifted +12 mm in Y does NOT imply every scan point should have a 12 mm nearest-surface distance.

Do not report surface-distance statistics as component displacement unless a valid component-level estimation method supports that conclusion.

---

## 2. Current Architecture

The project intentionally separates Rhino-side CAD automation from the external analysis engine.

### Rhino layer

Location:

`rhino_scripts/`

Environment:

- Rhino 7
- IronPython 2.7
- RhinoCommon
- rhinoscriptsyntax

Responsibilities:

- Create or inspect CAD geometry
- Access Rhino objects and metadata
- Generate controlled synthetic test geometry
- Export geometry and component metadata
- Import or visualize analysis results later

Important:

Rhino 7 scripts MUST remain compatible with IronPython 2.7.

Do not introduce:

- Python 3-only syntax
- f-strings
- pathlib
- modern type annotations
- pip-only dependencies
- libraries unavailable inside Rhino 7

unless the code is explicitly intended for the external Python engine rather than Rhino.

---

### External analysis engine

Location:

`src/`

Environment:

- Python 3.12
- Open3D
- NumPy
- additional scientific/ML libraries only when justified

Responsibilities:

- Read exported CAD/reference data
- Read point-cloud data
- Perform geometric analysis
- Registration
- Point-to-mesh distance calculations
- Component localization
- Component pose/deformation estimation
- QA metrics
- Later: reporting, APIs, ML and AI functionality

Do not move heavy numerical or machine-learning processing into Rhino unless there is a strong technical reason.

---

## 3. Current Repository Intent

Expected structure:

```text
ShipQA/
├── AGENTS.md
├── docs/
│   └── PROJECT_CONTEXT.md
├── rhino_scripts/
│   ├── 01_build_nominal_panel.py
│   ├── 02_build_asbuilt_test.py
│   ├── 03_generate_synthetic_scan.py
│   ├── 04_export_nominal_mesh.py
│   └── 05_export_component_manifest.py
├── src/
│   └── qa_engine.py
├── data/
├── tests/
└── .venv/

Do not reorganize the repository substantially without a clear reason.
Do not modify .venv.
Generated scan and analysis data should remain separate from source code.
4. Development Priorities
The order of priority is:
engineering correctness
→ reproducibility
→ validation
→ robustness
→ architecture
→ automation
→ UX
→ AI/ML
Do not reverse this order merely to make the project appear more advanced.
A deterministic geometric solution is preferred when it solves the problem reliably.
AI should only be introduced where deterministic geometry becomes inadequate, for example:
- semantic segmentation of complex real-world point clouds
- automatic component recognition
- correspondence between scan components and CAD components
- anomaly classification
- document/engineering knowledge assistance
Do not add an LLM simply to label the project as AI-powered.
5. Testing Philosophy
Synthetic defects are ground-truth experiments.
When a controlled defect is introduced, its exact value must be known.
Example:
STF_03 nominal center Y = 1200 mm
Injected translation = -8 mm
Expected as-built center ≈ 1192 mm

The analysis pipeline should attempt to recover that value independently.
Never use the known answer inside the detection or estimation algorithm.
Ground-truth metadata may be used only for validation after the estimate has been produced.
This separation is critical.
6. Avoid Data Leakage
Synthetic test scripts may store metadata such as:
KNOWN_SHIFT_Y_MM
KNOWN_DEFECT

The QA engine must NOT use these fields to determine the detected defect.
They exist only to compare predicted values against known ground truth.
If any algorithm reads ground-truth metadata before producing its estimate, treat that as a design flaw.
7. Numerical Engineering Rules
Always work explicitly in millimetres unless otherwise documented.
Do not silently convert units.
For every significant geometric calculation:
- state or preserve the unit
- prefer robust statistics over raw extrema when scanner noise is present
- distinguish measurement noise from actual manufacturing deviation
- avoid interpreting a single outlier as a structural defect
- retain enough data to reproduce the result
Metrics such as mean, median, P95, P99 and maximum may have different engineering meanings.
Do not use one interchangeably with another.
8. Registration
Future CAD-to-scan registration is a critical part of the product.
Do not assume scan and CAD coordinates will already be aligned in real-world data.
Development should eventually distinguish:
- coarse/global alignment
- fine registration
- local/component registration
- actual manufacturing deviation
Registration error must not be mistaken for manufacturing error.
Any implementation of ICP or another registration algorithm must expose useful quality/error metrics.
9. Component-Level QA
The intended product is not merely a point-cloud heatmap.
The useful output should eventually be engineering-oriented, such as:
Component: STF_03
Status: FAIL
Nominal Y: 1200.000 mm
Observed Y: 1192.180 mm
Estimated transverse shift: -7.820 mm
Tolerance: ±3.000 mm

Whenever possible, prefer component-level conclusions over millions of unstructured point deviations.
10. Rhino Metadata
Preserve engineering identifiers such as:
- PART_ID
- ASSEMBLY_ID
- MODEL_TYPE
- component bounding boxes
- nominal centers
Do not replace component identity with assumptions based only on object ordering.
Object order is not a reliable engineering identifier.
11. Code Quality
Prefer small, understandable modules over one large script.
Avoid unnecessary abstraction during the prototype stage.
However, do not duplicate important geometric logic across multiple files.
Functions should have clear engineering meaning.
Examples:
load_scan()load_nominal_mesh()compute_surface_deviation()identify_candidate_component()estimate_component_pose()evaluate_tolerance()


Names such as do_stuff() or process_data() should be avoided for core engineering operations.
12. Changes to Existing Behavior
Before changing an algorithm that currently works:
1. understand what engineering assumption it represents
2. preserve the existing validation case
3. add or update a test
4. compare results before and after the change
Do not perform large refactors simply for stylistic reasons.
Do not silently change thresholds, tolerances, coordinate conventions, units, sampling density, or noise assumptions.
13. Current Stage
The project is currently an early technical prototype.
The immediate objective is NOT:
- polished UI
- SaaS deployment
- authentication
- cloud infrastructure
- dashboards
- LLM integration
- full vessel-scale processing
The immediate objective is to prove that the geometry/measurement pipeline can reliably recover known dimensional defects under progressively harder conditions.
14. Progression Strategy
Increase difficulty gradually.
Start with controlled cases such as:
- single component translation
- positive and negative displacement
- different displacement magnitudes
- different scanner noise levels
Then progress toward:
- rotation
- deformation
- multiple simultaneous defects
- missing components
- partial scans
- occlusion
- outliers
- imperfect registration
- real point-cloud data
Do not jump directly from one clean synthetic test to a complex real ship block and assume failures are caused by AI or model quality.
15. Product Direction
The eventual product should support a workflow conceptually similar to:
Nominal CAD / production model
            +
As-built scan / point cloud
            ↓
      registration
            ↓
geometric deviation analysis
            ↓
component identification
            ↓
pose / deformation estimation
            ↓
tolerance evaluation
            ↓
Rhino visualization + engineering QA report

Later versions may integrate directly with shipyard CAD/PDM/PLM and scanning workflows.
Local/on-premise deployment should remain a serious design consideration because shipyard and defense-related geometry may be sensitive.
16. Working With Codex
Before making substantial changes:
- read this file
- read docs/PROJECT_CONTEXT.md
- inspect the existing implementation
- understand the current milestone
Do not assume missing project details.
If a proposed change could invalidate previous measurements or engineering assumptions, explain the issue before implementing it.
When asked to implement a milestone:
- modify the minimum necessary files
- preserve working behavior
- run available validation/tests
- report exactly what changed
- report any assumptions
- report failures honestly
Do not claim that an engineering feature works unless it has actually been validated.
17. Decision Standard
When multiple implementations are possible, prefer the one that is:
1. technically correct
2. measurable
3. testable
4. explainable to an engineer
5. robust to realistic data
6. maintainable
7. useful in a real shipyard workflow
Novelty alone is not sufficient.
The objective is not to make ShipQA look sophisticated.
The objective is to make ShipQA technically defensible and practically useful.