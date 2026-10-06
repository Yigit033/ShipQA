"""Predict all cases first, then evaluate. No experiment labels enter the engine."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys


ENGINE = Path(__file__).with_name("qa_engine.py")


def read_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def write_json_new(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def predict_case(scan, mesh, manifest, output_dir):
    """Only these four paths cross the subprocess boundary."""
    completed = subprocess.run(
        [sys.executable, "-B", str(ENGINE), "--scan", str(scan), "--mesh", str(mesh),
         "--manifest", str(manifest), "--output-dir", str(output_dir)],
        capture_output=True, text=True, check=False,
    )
    # Import/startup failures may occur before the engine can write its own report.
    if not (output_dir / "prediction.json").exists():
        output_dir.mkdir(exist_ok=True)
        with (output_dir / "analysis.log").open("a", encoding="utf-8") as log:
            log.write(completed.stdout + completed.stderr)
        write_json_new(output_dir / "prediction.json", {
            "status": "error", "error": "Engine exited without a prediction",
            "exit_code": completed.returncode,
        })


def category_for_shift(shift):
    # Evaluation policy only. Never called by qa_engine or during prediction.
    if shift == 0:
        return "zero_defect_control"
    if shift in (-4, 4):
        return "sub_threshold_characterization"
    if shift in (-16, -12, -8, 8, 12, 16):
        return "detectable_translation"
    raise ValueError("Shift is outside this approved validation sweep")


def evaluate_case(case_id, prediction, truth):
    if truth.get("units") != "mm":
        raise ValueError("Ground truth units must be mm")
    shift = float(truth["injected_shift_y_mm"])
    category = category_for_shift(shift)
    if truth["category"] != category:
        raise ValueError("Ground truth category and shift disagree")
    target = truth["injected_component"]
    if not isinstance(target, str) or not target:
        raise ValueError("Ground truth must explicitly identify the injected component")
    p = prediction if isinstance(prediction, dict) else ({"status": "invalid_prediction"} if prediction is not None else {})
    status = p.get("status", "missing_prediction")
    completed = status in ("estimated", "no_candidates", "no_component_points")
    candidates = p.get("candidate_count")
    component = p.get("detected_component")
    estimate = p.get("estimated_shift_y_mm")
    points = p.get("points_used")
    if completed:
        valid_fields = (
            isinstance(candidates, int) and not isinstance(candidates, bool) and candidates >= 0
            and isinstance(points, int) and not isinstance(points, bool) and points >= 0
            and (component is None or isinstance(component, str))
            and (estimate is None or (isinstance(estimate, (int, float))
                                     and not isinstance(estimate, bool) and math.isfinite(estimate)))
        )
        if status == "estimated":
            valid_fields = valid_fields and estimate is not None and bool(component) and points > 0 and candidates > 0
        elif status == "no_candidates":
            valid_fields = valid_fields and candidates == 0 and component is None and estimate is None
        elif status == "no_component_points":
            valid_fields = valid_fields and candidates > 0 and bool(component) and points == 0 and estimate is None
        if not valid_fields:
            status, completed = "invalid_prediction", False
    detected = (candidates > 0) if completed else None
    correct = (component == target) if completed and component else None
    valid_estimate = completed and status == "estimated" and correct is True and category != "zero_defect_control"
    signed_error = float(estimate) - shift if valid_estimate else None
    false_positive = bool(component) if completed and category == "zero_defect_control" else None
    if not completed:
        outcome = status
    elif category == "zero_defect_control":
        outcome = "false_positive" if detected or component else "expected_no_detection"
    elif category == "sub_threshold_characterization":
        outcome = "characterized"
    elif not detected:
        outcome = "no_detection"
    elif not component:
        outcome = "no_identification"
    elif not correct:
        outcome = "incorrect_component"
    elif not valid_estimate:
        outcome = "missing_estimate"
    else:
        outcome = "estimated"
    return {
        "case_id": case_id, "category": category, "injected_component": target,
        "injected_shift_y_mm": shift, "prediction_status": status, "outcome": outcome,
        "prediction_completed": completed, "detected": detected,
        "candidate_count": candidates, "detected_component": component,
        "correct_component": correct, "false_positive_component": false_positive,
        "predicted_shift_y_mm": estimate, "points_used": points,
        "surface_p95_mm": p.get("surface_p95_mm"), "surface_p99_mm": p.get("surface_p99_mm"),
        "signed_error_mm": signed_error,
        "absolute_error_mm": abs(signed_error) if signed_error is not None else None,
        "relative_absolute_error_percent": 100 * abs(signed_error) / abs(shift) if signed_error is not None and shift != 0 else None,
        "valid_estimate": bool(valid_estimate),
        "eligible_for_estimation_metrics": bool(valid_estimate and category == "detectable_translation"),
    }


def aggregate_results(rows):
    detectable = [r for r in rows if r["category"] == "detectable_translation"]
    controls = [r for r in rows if r["category"] == "zero_defect_control"]
    subthreshold = [r for r in rows if r["category"] == "sub_threshold_characterization"]

    def detection_counts(group):
        return {
            "cases": len(group),
            "completed_predictions": sum(r["prediction_completed"] for r in group),
            "detected_cases": sum(r["detected"] is True for r in group),
            "identified_cases": sum(r["prediction_completed"] and bool(r["detected_component"]) for r in group),
            "correct_component_identifications": sum(r["correct_component"] is True for r in group),
            "incorrect_component_identifications": sum(r["correct_component"] is False for r in group),
            "no_detection_cases": sum(r["detected"] is False for r in group),
            "prediction_failure_cases": sum(not r["prediction_completed"] for r in group),
        }

    detection = detection_counts(detectable)
    detection["expected_detectable_cases"] = len(detectable)
    detection["cases_without_valid_estimate"] = sum(not r["valid_estimate"] for r in detectable)
    control = {
        "cases": len(controls),
        "completed_predictions": sum(r["prediction_completed"] for r in controls),
        "false_positive_detection_cases": sum(r["detected"] is True for r in controls),
        "false_positive_component_cases": sum(r["false_positive_component"] is True for r in controls),
        "correct_no_detection_cases": sum(r["outcome"] == "expected_no_detection" for r in controls),
        "prediction_failure_cases": sum(not r["prediction_completed"] for r in controls),
    }
    eligible = [r for r in detectable if r["eligible_for_estimation_metrics"]]
    errors = [r["signed_error_mm"] for r in eligible]
    return {
        "units": "mm", "total_cases": len(rows),
        "detectable_translation_detection": detection,
        "zero_defect_control": control,
        "sub_threshold_characterization": detection_counts(subthreshold),
        "estimation": {
            "population": "detectable translations with correct component and finite displacement estimate",
            "expected_detectable_cases": len(detectable), "valid_estimates": len(errors),
            "excluded_cases": [{"case_id": r["case_id"], "reason": r["outcome"]}
                               for r in detectable if not r["eligible_for_estimation_metrics"]],
            "mean_signed_error_mm": sum(errors) / len(errors) if errors else None,
            "mae_mm": sum(abs(e) for e in errors) / len(errors) if errors else None,
            "maximum_absolute_error_mm": max(abs(e) for e in errors) if errors else None,
        },
    }


def run_validation(run_dir, predictor=predict_case, output_dir=None):
    run_dir = Path(run_dir).resolve()
    output_root = run_dir if output_dir is None else Path(output_dir).resolve()
    metadata = read_json(run_dir / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if (metadata["generation_status"] != "complete" or not case_ids
            or metadata["completed_case_ids"] != case_ids or len(set(case_ids)) != len(case_ids)):
        raise ValueError("Generation is incomplete or case IDs are duplicated")
    if any(not isinstance(c, str) or not c.startswith("case_") or not c[5:].isdigit() for c in case_ids):
        raise ValueError("Invalid neutral case ID")
    if metadata["units"] != "mm" or (metadata["point_count"], metadata["noise_sigma_mm"], metadata["seed"]) != (30000, 0.35, 42):
        raise ValueError("Run settings do not match the approved sweep")
    if set(metadata["reference_sha256"]) != {"nominal_reference.obj", "component_manifest.json"}:
        raise ValueError("Both frozen reference hashes are required")
    for name, expected_hash in metadata["reference_sha256"].items():
        if name not in ("nominal_reference.obj", "component_manifest.json"):
            raise ValueError("Unexpected reference filename")
        if file_hash(run_dir / "reference" / name) != expected_hash:
            raise ValueError("Frozen nominal reference has changed")
    cases = [run_dir / "cases" / case_id for case_id in case_ids]
    for case in cases:
        if not (case / "input" / "scan.xyz").is_file():
            raise FileNotFoundError(case / "input" / "scan.xyz")
        if (output_root / "cases" / case.name / "prediction").exists():
            raise FileExistsError("Prediction already exists; refusing to overwrite: {}".format(case))
    for name in ("summary.csv", "aggregate.json", "prediction_phase.json"):
        if (output_root / name).exists():
            raise FileExistsError(output_root / name)
    if output_dir is not None:
        output_root.mkdir(parents=True, exist_ok=False)

    # Phase 1: no ground-truth reads, including for cases already predicted.
    for case in cases:
        predictor(case / "input" / "scan.xyz",
                  run_dir / "reference" / "nominal_reference.obj",
                  run_dir / "reference" / "component_manifest.json",
                  output_root / "cases" / case.name / "prediction")
    write_json_new(output_root / "prediction_phase.json", {
        "status": "complete", "case_ids": case_ids,
        "source_run": str(run_dir),
        "engine_sha256": file_hash(ENGINE), "evaluator_sha256": file_hash(Path(__file__)),
        "distance_kernel_sha256": file_hash(ENGINE.with_name("surface_distance.py")),
    })

    # Phase 2: only after ALL prediction invocations have completed.
    rows = []
    for case in cases:
        path = output_root / "cases" / case.name / "prediction" / "prediction.json"
        try:
            prediction = read_json(path) if path.is_file() else None
        except (ValueError, OSError):
            prediction = {"status": "invalid_prediction"}
        if prediction is not None and not isinstance(prediction, dict):
            prediction = {"status": "invalid_prediction"}
        truth = read_json(case / "ground_truth" / "ground_truth.json")
        scan_hash = file_hash(case / "input" / "scan.xyz")
        if truth["scan_sha256"] != scan_hash:
            raise ValueError("Ground truth is not bound to this scan: {}".format(case.name))
        if prediction and prediction.get("status") not in ("error", "invalid_prediction"):
            expected = {"scan": scan_hash,
                        "mesh": file_hash(run_dir / "reference" / "nominal_reference.obj"),
                        "manifest": file_hash(run_dir / "reference" / "component_manifest.json")}
            if (prediction.get("input_sha256") != expected
                    or prediction.get("scan_point_count") != metadata["point_count"]):
                prediction = dict(prediction, status="input_mismatch")
        rows.append(evaluate_case(case.name, prediction, truth))
    with (output_root / "summary.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    aggregate = aggregate_results(rows)
    write_json_new(output_root / "aggregate.json", aggregate)
    return aggregate


def main():
    parser = argparse.ArgumentParser(description="Evaluate a completed Rhino Y-translation batch")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, help="New analysis directory; reuse scans without replacing previous results")
    args = parser.parse_args()
    print(json.dumps(run_validation(args.run_dir, output_dir=args.output_dir), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
