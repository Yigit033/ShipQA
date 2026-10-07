"""Predict all local rotation cases before reading and scoring ground truth."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from validate_y_translations import file_hash, predict_case, read_json, write_json_new


def rotation_error_degrees(predicted, expected):
    delta = np.asarray(predicted, dtype=np.float64) @ np.asarray(expected, dtype=np.float64).T
    value = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(value))


def evaluate_case(case_id, prediction, truth):
    if truth.get("units") != "mm" or truth.get("angle_units") != "degrees":
        raise ValueError("Rotation ground truth must use millimetres and degrees")
    injected = np.asarray(truth["injected_rotation_xyz_deg"], dtype=np.float64)
    expected_matrix = np.asarray(truth["rotation_matrix_nominal_to_observed"], dtype=np.float64)
    if injected.shape != (3,) or expected_matrix.shape != (3, 3):
        raise ValueError("Invalid rotation ground truth")
    category = truth["category"]
    if category not in ("zero_defect_control", "sub_threshold_characterization",
                        "detectable_rotation"):
        raise ValueError("Unknown rotation scoring category")
    target = truth["injected_component"]
    p = prediction if isinstance(prediction, dict) else {}
    status = p.get("status", "missing_prediction")
    completed = status in ("estimated", "no_candidates", "no_component_points")
    candidates = p.get("candidate_count")
    component = p.get("detected_component")
    detected = candidates > 0 if completed and isinstance(candidates, int) else None
    correct = component == target if completed and component else None
    local = p.get("component_rotation", {})
    predicted_matrix = local.get("rotation_matrix_nominal_to_observed")
    predicted_euler = local.get("estimated_rotation_xyz_deg")
    valid_rotation = False
    angular_error = None
    axis_errors = [None, None, None]
    if correct is True and local.get("status") == "estimated_provisional":
        try:
            matrix = np.asarray(predicted_matrix, dtype=np.float64)
            euler = np.asarray(predicted_euler, dtype=np.float64)
            valid_rotation = (matrix.shape == (3, 3) and euler.shape == (3,)
                              and np.isfinite(matrix).all() and np.isfinite(euler).all())
            if valid_rotation:
                angular_error = rotation_error_degrees(matrix, expected_matrix)
                axis_errors = (euler - injected).tolist()
        except (TypeError, ValueError):
            valid_rotation = False
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
    elif not valid_rotation:
        outcome = "missing_rotation_estimate"
    else:
        outcome = "estimated"
    return {
        "case_id": case_id, "category": category,
        "injected_rx_deg": float(injected[0]), "injected_ry_deg": float(injected[1]),
        "injected_rz_deg": float(injected[2]), "prediction_status": status,
        "outcome": outcome, "prediction_completed": completed, "detected": detected,
        "candidate_count": candidates, "detected_component": component,
        "correct_component": correct,
        "predicted_rx_deg": predicted_euler[0] if valid_rotation else None,
        "predicted_ry_deg": predicted_euler[1] if valid_rotation else None,
        "predicted_rz_deg": predicted_euler[2] if valid_rotation else None,
        "signed_error_rx_deg": axis_errors[0], "signed_error_ry_deg": axis_errors[1],
        "signed_error_rz_deg": axis_errors[2],
        "angular_error_deg": angular_error, "valid_rotation_estimate": valid_rotation,
        "rotation_points_used": local.get("observed_points_used"),
        "rotation_fitness": (local.get("stages") or [{}])[-1].get("fitness"),
        "rotation_rmse_mm": (local.get("stages") or [{}])[-1].get("inlier_rmse_mm"),
        "surface_p95_mm": p.get("surface_p95_mm"),
        "surface_p99_mm": p.get("surface_p99_mm"),
    }


def aggregate_results(rows):
    detectable = [row for row in rows if row["category"] == "detectable_rotation"]
    subthreshold = [row for row in rows if row["category"] == "sub_threshold_characterization"]
    controls = [row for row in rows if row["category"] == "zero_defect_control"]
    valid = [row for row in detectable if row["valid_rotation_estimate"]
             and row["correct_component"] is True]
    errors = [row["angular_error_deg"] for row in valid]

    def counts(group):
        return {"cases": len(group),
                "detected_cases": sum(row["detected"] is True for row in group),
                "correct_component_identifications": sum(
                    row["correct_component"] is True for row in group),
                "no_detection_cases": sum(row["detected"] is False for row in group),
                "prediction_failure_cases": sum(not row["prediction_completed"] for row in group)}
    return {
        "angle_units": "degrees", "total_cases": len(rows),
        "detectable_rotation": counts(detectable),
        "sub_threshold_characterization": counts(subthreshold),
        "zero_control": {"cases": len(controls),
                         "false_positive_cases": sum(row["outcome"] == "false_positive"
                                                     for row in controls)},
        "rotation_estimation": {
            "expected_detectable_cases": len(detectable),
            "valid_estimates": len(valid),
            "excluded_cases": [{"case_id": row["case_id"], "reason": row["outcome"]}
                               for row in detectable if row not in valid],
            "mean_angular_error_deg": sum(errors) / len(errors) if errors else None,
            "maximum_angular_error_deg": max(errors) if errors else None,
            "worst_case": max(valid, key=lambda row: row["angular_error_deg"])["case_id"]
            if valid else None,
        },
    }


def run_validation(run_dir, predictor=predict_case, output_dir=None):
    run_dir = Path(run_dir).resolve()
    output_root = run_dir if output_dir is None else Path(output_dir).resolve()
    metadata = read_json(run_dir / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if (metadata.get("experiment") != "independent_component_rotation"
            or metadata.get("generation_status") != "complete"
            or metadata.get("completed_case_ids") != case_ids):
        raise ValueError("Rotation generation is incomplete or incompatible")
    if (metadata.get("point_count"), metadata.get("noise_sigma_mm"),
            metadata.get("seed")) != (30000, 0.35, 42):
        raise ValueError("Rotation run settings do not match the approved sweep")
    reference = run_dir / "reference"
    mesh = reference / "nominal_reference.obj"
    manifest = reference / "component_manifest.json"
    for name, expected in metadata["reference_sha256"].items():
        if file_hash(reference / name) != expected:
            raise ValueError("Frozen reference changed")
    cases = [run_dir / "cases" / case_id for case_id in case_ids]
    if output_dir is not None:
        output_root.mkdir(parents=True, exist_ok=False)
    for case in cases:
        destination = output_root / "cases" / case.name / "prediction"
        if destination.exists():
            raise FileExistsError(destination)
        predictor(case / "input" / "scan.xyz", mesh, manifest, destination)
    write_json_new(output_root / "prediction_phase.json", {
        "status": "complete", "case_ids": case_ids, "source_run": str(run_dir)})
    rows = []
    for case in cases:
        path = output_root / "cases" / case.name / "prediction" / "prediction.json"
        try:
            prediction = read_json(path)
        except (OSError, ValueError):
            prediction = None
        truth = read_json(case / "ground_truth" / "ground_truth.json")
        if truth["scan_sha256"] != file_hash(case / "input" / "scan.xyz"):
            raise ValueError("Ground truth is not bound to scan")
        rows.append(evaluate_case(case.name, prediction, truth))
    with (output_root / "summary.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    aggregate = aggregate_results(rows)
    write_json_new(output_root / "aggregate.json", aggregate)
    return aggregate


def main():
    parser = argparse.ArgumentParser(description="Evaluate a Rhino component-rotation batch")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(run_validation(args.run_dir, output_dir=args.output_dir),
                     indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
