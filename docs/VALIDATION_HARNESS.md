# Controlled Y-translation validation

The nine-case Rhino dataset was generated and validated on 2026-10-06. A subsequent
primitive-level distance correction was checked against the preserved regression
and all nine scans. See [SURFACE_DISTANCE_INVESTIGATION.md](SURFACE_DISTANCE_INVESTIGATION.md)
for the confirmed root cause, before/after results and remaining limits. The steps
below describe generating and evaluating a new batch; existing scans need not be regenerated.

## Preserved regression

`data/regression/stf03_negative_8/` preserves the actual Rhino-generated -8 mm scan,
nominal reference, manifest, pre-interface engine and generator source snapshots,
checksums, candidate points and full-precision baseline result. The baseline and
new interface both identify STF_03 and estimate -8.126176 mm using 1,945 points.
The archived sources are historical evidence, not active implementations.

## Next action in Rhino 7

1. Open a **copy** of the existing nominal test-panel document, in millimetres.
   It must contain one nominal plate and five complete nominal T-stiffeners with
   their original PART_ID and ASSEMBLY_ID metadata. Do not run script 01 again
   in a document already containing the nominal panel: it would duplicate it.
2. Use `RunPythonScript` and select
   `C:\active_projects\ShipQA\rhino_scripts\06_generate_y_validation_batch.py`.
3. Wait for `Generated all nine scans. External QA validation has NOT been run.`
   Copy the printed validation run directory and report it back for analysis.

No nine manual save dialogs are required. Each case rebuilds from the same nominal
object IDs. The batch stops on generation errors and records an incomplete/failed
run; it does not label partial output complete. Rerunning creates a new run, never
replaces the old one. Normal completion leaves the AS-BUILT layer at the final
+16 mm case. Batch sampling does not replace the displayed scan cloud; inspect
the exported XYZ files to view individual cases. Use the document copy to avoid
changing the working document's AS-BUILT state.

The existing scripts 02-05 remain runnable individually with `RunPythonScript`.
Script 02 still defaults to -8 mm; script 03 still defaults to the active as-built
layer, seed 42, its normal scan display, and its interactive save dialog.

## Files and provenance

```text
data/validation/<unique_run_id>/
    reference/
        nominal_reference.obj
        component_manifest.json
    provenance/                         # Rhino source snapshots
    run_metadata.json                   # settings, order, hashes, completion
    cases/
        case_001/
            input/scan.xyz
            ground_truth/ground_truth.json
            prediction/                 # created later by external Python
                prediction.json
                defect_candidates.xyz
                analysis.log
        ...
    prediction_phase.json               # written after all predictions finish
    summary.csv
    aggregate.json
```

The batch exports fresh nominal references from the same Rhino document before
creating cases. Case IDs carry no shift information. Ground truth explicitly
records the injected component, shift and category and is bound to the scan by
SHA-256. Run metadata records the nominal part bounds, captured sampling order,
Rhino version, source hashes, reference hashes and generation status. The per-case
generation record also contains the meshing triangle count, ordered triangle hash,
surface area, point count, noise sigma and seed.

Nominal geometry is assumed to be the current controlled panel. Generation checks
require its expected component identities and verify each copied part's translated
bounds using the Rhino document's existing absolute tolerance. This CAD generation
check does not alter the engine's detection gate or estimator and is not a general
geometry-equivalence test. Nominal units are checked, never converted silently.

## External prediction and evaluation

Only after Rhino generation completes, from the repository root:

```powershell
.\.venv\Scripts\python.exe -B src\validate_y_translations.py "data\validation\<run_id>"
```

The runner invokes the engine sequentially in separate processes. Each engine
invocation receives **only** the scan, nominal mesh, nominal manifest and prediction
output directory. No injected component, shift, case category, ground-truth path,
or run metadata is passed to it. All invocations finish before the evaluator opens
any ground-truth file. Input hashes bind predictions to the evaluated data.

Existing prediction directories and summary files are never overwritten. To rerun
analysis on the same inputs, use a new output directory:

```powershell
.\.venv\Scripts\python.exe -B src\validate_y_translations.py "data\validation\<run_id>" --output-dir "data\analysis\<new_analysis_id>"
```

Inputs and ground truth are still read from the original run. If analysis is
interrupted or rejects inconsistent data, preserve its artifacts and investigate;
the harness does not resume/replace results in an existing output directory.
Startup and engine errors are distinct from a successful no-candidate outcome.

For one archived input, or another geometry-only input, the engine also accepts:

```powershell
.\.venv\Scripts\python.exe -B src\qa_engine.py --scan <scan.xyz> --mesh <nominal.obj> --manifest <manifest.json> --output-dir <new_prediction_directory>
```

The original `.\.venv\Scripts\python.exe src\qa_engine.py` command still uses the
generic files in `data/` and prints its report. Structured prediction writes full
precision rather than extracting rounded values from console output. A successful
no-candidate case returns `status: no_candidates`, null component/displacement,
surface statistics and an empty candidates file. It does not invent a zero shift.

## Scoring policy

| Category | Injected shifts (mm) | Interpretation |
|---|---|---|
| Zero-defect control | 0 | No candidates/component is the expected outcome; report any candidate detection and any reported component as separate false-positive counts. |
| Sub-threshold characterization | -4, +4 | Report candidates, identification and any estimate; no estimate is not an estimator failure. |
| Detectable translation | -16, -12, -8, +8, +12, +16 | Evaluate detection, correct identity, availability of displacement and numerical error. |

Each CSV row contains the explicit injected shift/component/category, prediction
status, candidate count, detected component, identification correctness, estimation
point count, predicted shift, surface P95/P99, signed/absolute errors and eligibility
for the aggregate estimation metrics. Signed error is prediction minus injection.
Relative absolute error uses the magnitude of the injected shift and is undefined
at zero. Errors are not assigned to the target when another component was reported.

Aggregate estimation metrics apply **only to detectable translations with a correct
component identification and finite displacement estimate**. They report valid
estimate count against the total detectable-case count, every excluded case and its
reason, mean signed error (bias), MAE and maximum absolute error. Empty populations
produce null metrics. Missing predictions/execution failures are not classified as
no detection. The zero control and sub-threshold results are reported separately.
No displacement acceptance tolerance or automatic accuracy pass criterion is added.

## Fixed experiment settings and limits

- 30,000 scan points, Gaussian coordinate noise sigma 0.35 mm, seed 42 for each case.
- Existing area-weighted triangle sampling and random-draw sequence.
- Existing surface-deviation gate strictly greater than 5 mm.
- Corrected float64 exhaustive triangle distance primitive; method and source hash
  are recorded in each prediction. The mesh still comes from the nominal OBJ.
- Existing nominal-region margins, component identification and 2nd/98th percentile
  Y-envelope estimator. No compensation for the approximately -0.126 mm error.

The batch captures nominal part order once and passes the corresponding copied IDs
directly to the sampler. It does not trust enumeration of newly created Rhino IDs.
This explicit order may differ from the earlier manual scan's enumeration, so seed
42 alone does **not** guarantee the identical old sample or -0.126 mm error. The
archived scan remains the exact regression test. Reproducing sampling requires the
recorded part order, nominal geometry, meshing behavior, source and Rhino version;
the retained scans permit independent reanalysis without Rhino.

The first sweep characterizes one seeded sampling realization. A consistent error
across translations alone is not evidence of bias over independent scan samples.
Multiple seeds would be a later experiment, without tuning the current estimator.

## Tests

```powershell
.\.venv\Scripts\python.exe -B -W ignore::DeprecationWarning -m unittest discover -s tests -v
```

Tests cover scoring categories and denominators, malformed/missing predictions,
incorrect identification, geometry-only engine arguments, delayed ground-truth
reads, the actual archived -8 mm regression, unchanged mathematical expressions,
unchanged sampler draw order, metadata-independent sampling with a geometry test
double, fresh nominal copying, overwrite protection, batch orchestration and legacy
Python grammar. They do not substitute for running Rhino 7 against the actual model.
