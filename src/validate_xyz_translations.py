"""Run geometry-only predictions first, then score controlled XYZ translations."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from validate_y_translations import file_hash, predict_case, read_json, write_json_new


def evaluate_case(case_id, prediction, truth):
    if truth.get("units") != "mm":
        raise ValueError("Ground truth units must be mm")
    injected = np.asarray(truth["injected_translation_xyz_mm"], dtype=np.float64)
    if injected.shape != (3,) or not np.isfinite(injected).all():
        raise ValueError("Ground truth translation must be a finite XYZ vector")
    category = "zero_defect_control" if np.all(injected == 0.0) else "detectable_translation"
    if truth.get("category") != category:
        raise ValueError("Ground truth category and translation disagree")
    target = truth["injected_component"]
    p = prediction if isinstance(prediction, dict) else {}
    status = p.get("status", "missing_prediction")
    completed = status in ("estimated", "no_candidates", "no_component_points")
    candidates = p.get("candidate_count")
    component = p.get("detected_component")
    estimate_value = p.get("estimated_translation_xyz_mm")
    estimate = None
    if isinstance(estimate_value, list) and len(estimate_value) == 3:
        candidate = np.asarray(estimate_value, dtype=np.float64)
        if np.isfinite(candidate).all():
            estimate = candidate
    detected = candidates > 0 if completed and isinstance(candidates, int) else None
    correct = component == target if completed and component else None
    valid = bool(completed and status == "estimated" and correct is True
                 and estimate is not None and category == "detectable_translation")
    error = estimate - injected if valid else None
    if not completed:
        outcome = status
    elif category == "zero_defect_control":
        outcome = "false_positive" if detected or component else "expected_no_detection"
    elif not detected:
        outcome = "no_detection"
    elif not component:
        outcome = "no_identification"
    elif not correct:
        outcome = "incorrect_component"
    elif not valid:
        outcome = "missing_xyz_estimate"
    else:
        outcome = "estimated"
    return {
        "case_id": case_id, "category": category,
        "injected_component": target,
        "injected_x_mm": float(injected[0]), "injected_y_mm": float(injected[1]),
        "injected_z_mm": float(injected[2]), "prediction_status": status,
        "outcome": outcome, "prediction_completed": completed,
        "detected": detected, "candidate_count": candidates,
        "detected_component": component, "correct_component": correct,
        "predicted_x_mm": None if estimate is None else float(estimate[0]),
        "predicted_y_mm": None if estimate is None else float(estimate[1]),
        "predicted_z_mm": None if estimate is None else float(estimate[2]),
        "signed_error_x_mm": None if error is None else float(error[0]),
        "signed_error_y_mm": None if error is None else float(error[1]),
        "signed_error_z_mm": None if error is None else float(error[2]),
        "absolute_error_x_mm": None if error is None else float(abs(error[0])),
        "absolute_error_y_mm": None if error is None else float(abs(error[1])),
        "absolute_error_z_mm": None if error is None else float(abs(error[2])),
        "vector_error_mm": None if error is None else float(np.linalg.norm(error)),
        "points_used": p.get("points_used"),
        "pose_points_used": p.get("component_pose", {}).get("pose_points_used"),
        "surface_p95_mm": p.get("surface_p95_mm"),
        "surface_p99_mm": p.get("surface_p99_mm"), "valid_estimate": valid,
    }


def aggregate_results(rows):
    detectable = [row for row in rows if row["category"] == "detectable_translation"]
    controls = [row for row in rows if row["category"] == "zero_defect_control"]
    valid = [row for row in detectable if row["valid_estimate"]]
    aggregate = {
        "units": "mm", "total_cases": len(rows),
        "detection": {
            "expected_detectable_cases": len(detectable),
            "actually_detected_cases": sum(row["detected"] is True for row in detectable),
            "correct_component_identifications": sum(
                row["correct_component"] is True for row in detectable),
            "no_detection_cases": sum(row["detected"] is False for row in detectable),
            "prediction_failure_cases": sum(not row["prediction_completed"] for row in detectable),
            "zero_control_false_positives": sum(
                row["outcome"] == "false_positive" for row in controls),
        },
        "estimation": {
            "expected_detectable_cases": len(detectable), "valid_estimates": len(valid),
            "excluded_cases": [{"case_id": row["case_id"], "reason": row["outcome"]}
                               for row in detectable if not row["valid_estimate"]],
        },
    }
    for index, axis in enumerate("xyz"):
        errors = [row["signed_error_{}_mm".format(axis)] for row in valid]
        aggregate["estimation"][axis] = {
            "mean_signed_error_mm": sum(errors) / len(errors) if errors else None,
            "mae_mm": sum(abs(value) for value in errors) / len(errors) if errors else None,
            "maximum_absolute_error_mm": max(abs(value) for value in errors) if errors else None,
        }
    vector_errors = [row["vector_error_mm"] for row in valid]
    aggregate["estimation"]["vector"] = {
        "mae_mm": sum(vector_errors) / len(vector_errors) if vector_errors else None,
        "maximum_error_mm": max(vector_errors) if vector_errors else None,
        "worst_case": (max(valid, key=lambda row: row["vector_error_mm"])["case_id"]
                       if valid else None),
    }
    return aggregate


def run_validation(run_dir, predictor=predict_case, output_dir=None):
    run_dir = Path(run_dir).resolve()
    output_root = run_dir if output_dir is None else Path(output_dir).resolve()
    metadata = read_json(run_dir / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if (metadata.get("experiment") != "independent_xyz_component_translation"
            or metadata.get("generation_status") != "complete"
            or metadata.get("completed_case_ids") != case_ids):
        raise ValueError("XYZ generation is incomplete or incompatible")
    if (metadata.get("units"), metadata.get("point_count"),
            metadata.get("noise_sigma_mm"), metadata.get("seed")) != (
                "mm", 30000, 0.35, 42):
        raise ValueError("XYZ run settings do not match the approved sweep")
    reference = run_dir / "reference"
    mesh = reference / "nominal_reference.obj"
    manifest = reference / "component_manifest.json"
    for name, expected in metadata["reference_sha256"].items():
        if file_hash(reference / name) != expected:
            raise ValueError("Frozen nominal reference has changed")
    cases = [run_dir / "cases" / case_id for case_id in case_ids]
    if output_dir is not None:
        output_root.mkdir(parents=True, exist_ok=False)
    for case in cases:
        destination = output_root / "cases" / case.name / "prediction"
        if destination.exists():
            raise FileExistsError(destination)
        # No truth path, category, expected component or expected axis crosses this boundary.
        predictor(case / "input" / "scan.xyz", mesh, manifest, destination)
    write_json_new(output_root / "prediction_phase.json", {
        "status": "complete", "case_ids": case_ids,
        "source_run": str(run_dir), "engine_input_paths": ["scan", "mesh", "manifest"],
    })

    rows = []
    for case in cases:
        prediction_path = output_root / "cases" / case.name / "prediction" / "prediction.json"
        try:
            prediction = read_json(prediction_path)
        except (OSError, ValueError):
            prediction = None
        truth = read_json(case / "ground_truth" / "ground_truth.json")
        if truth["scan_sha256"] != file_hash(case / "input" / "scan.xyz"):
            raise ValueError("Ground truth is not bound to scan: {}".format(case.name))
        rows.append(evaluate_case(case.name, prediction, truth))
    with (output_root / "summary.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    aggregate = aggregate_results(rows)
    write_json_new(output_root / "aggregate.json", aggregate)
    return aggregate


def main():
    parser = argparse.ArgumentParser(description="Evaluate a completed Rhino XYZ translation batch")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(run_validation(args.run_dir, output_dir=args.output_dir),
                     indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
