# Real-data engineering pilot

The next ShipQA milestone is one supervised real-data pilot on a bounded stiffened
panel or assembly. Its purpose is to determine where the validated synthetic
geometry pipeline survives real acquisition conditions and where it fails. The
pilot is measurement evidence, not production acceptance certification.

## Minimum pilot scope

Choose one accessible structure with:

- a frozen nominal CAD revision;
- stable engineering identifiers for the plate and stiffeners;
- at least one component with independently measured position and orientation;
- scan coverage of the component web, flange, and surrounding reference plate;
- surveyed targets, a scanner-pose transform, or widely separated corresponding
  points for coarse scan-to-CAD alignment;
- millimetre units throughout.

The first pilot should emphasize Y/Z translation and rotation. Longitudinal X can
be evaluated only when both component ends or other X-normal surfaces are actually
measured.

## Immutable input package

Use a unique directory and retain the original scanner export:

```text
data/pilot/<pilot_id>/
    source/
        original_scan.<scanner-format>
        source_metadata.json
    reference/
        nominal_reference.obj
        component_manifest.json
        cad_revision.json
    input/
        scan.xyz
        conversion_metadata.json
    control/
        initial_alignment.json
    prediction/
    independent_evidence/
        measurements.json
        measurement_method.md
```

`scan.xyz` must contain finite X Y Z coordinates in millimetres. Preserve the
original E57, PTS, LAS, PLY, or vendor file unchanged. Record the conversion tool,
version, source/output hashes, unit conversion, filters, and point-count change.
ShipQA should not receive colour, intensity, normals, or ground-truth measurements
as prediction inputs during this first pilot.

The nominal OBJ must use `g PART_ID` group records. The manifest must preserve
`PART_ID`, `ASSEMBLY_ID`, nominal component centres, and bounding boxes. Its CAD
revision must match the physical structure that was scanned.

## Coarse alignment

For real data, provide one of the existing `initial_alignment.json` forms:

- a proper 4x4 scanner/survey transform mapping scan coordinates to CAD; or
- at least three non-collinear corresponding scan/CAD points.

Four or more widely distributed surveyed controls are preferable because they
permit an independent residual check and reduce repeated-geometry ambiguity. A
verified shared coordinate system may use an identity transform with its survey
provenance recorded.

## Prediction command

The prediction engine receives only scan, nominal reference, component manifest,
coarse alignment, and a new output directory:

```powershell
.\.venv\Scripts\python.exe -B src\qa_engine.py `
  --scan data\pilot\<pilot_id>\input\scan.xyz `
  --mesh data\pilot\<pilot_id>\reference\nominal_reference.obj `
  --manifest data\pilot\<pilot_id>\reference\component_manifest.json `
  --initial-alignment data\pilot\<pilot_id>\control\initial_alignment.json `
  --register `
  --output-dir data\pilot\<pilot_id>\prediction
```

Use `--filter-isolated-outliers` only when the raw scan contains demonstrated
isolated environmental returns. Always preserve and report the removed count and
retained fraction.

## Review order

1. Verify file hashes, units, CAD revision, point count, and bounds.
2. Review coarse-control residuals and registration transform integrity.
3. Require the provisional registration quality gate to pass before component QA.
4. Review retained fraction, coverage, post-registration P95, and fine fitness.
5. Review raw and spatially supported defect candidates.
6. Review detected component and local-pose fitness/RMSE.
7. Freeze `prediction.json`.
8. Only then open `independent_evidence/measurements.json` and calculate errors.

The current registration gates were qualified on synthetic 0.35 mm-noise data and
remain provisional for field use. A pass is permission to inspect the component
result, not proof that the result satisfies yard or classification tolerances.

## Pilot outcome

The pilot report must state:

- whether registration was accepted or blocked and why;
- component detection and identity;
- observable and unobservable translation axes;
- rotation and translation estimates with quality metrics;
- comparison with independent measurements and their uncertainty;
- occlusion, density, clutter, reflective-surface, and coverage observations;
- every manual preprocessing operation;
- whether any result is suitable for engineering assistance, re-scan guidance, or
  only further algorithm development.

No estimator threshold or acceptance gate should be tuned until the original
prediction and independent measurement comparison are preserved and the failure's
geometric cause is identified.
