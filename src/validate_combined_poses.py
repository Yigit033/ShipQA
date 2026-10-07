"""Predict every combined-pose case before reading and scoring ground truth."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from validate_component_rotations import rotation_error_degrees
from validate_y_translations import file_hash, predict_case, read_json, write_json_new


AXES = "xyz"


def finite_vector(value):
    try:
        vector = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    return vector if vector.shape == (3,) and np.isfinite(vector).all() else None


def evaluate_case(case_id, prediction, truth):
    if truth.get("units") != "mm" or truth.get("angle_units") != "degrees":
        raise ValueError("Combined-pose ground truth must use mm and degrees")
    injected_translation = finite_vector(truth.get("injected_translation_xyz_mm"))
    injected_rotation = finite_vector(truth.get("injected_rotation_xyz_deg"))
    expected_rotation_matrix = np.asarray(
        truth.get("rotation_matrix_nominal_to_observed"), dtype=np.float64)
    if (injected_translation is None or injected_rotation is None
            or expected_rotation_matrix.shape != (3, 3)
            or not np.isfinite(expected_rotation_matrix).all()):
        raise ValueError("Invalid combined-pose ground truth")
    category = truth.get("category")
    if category not in ("zero_defect_control", "detectable_combined_pose"):
        raise ValueError("Unknown combined-pose scoring category")
    required_axes = truth.get("required_translation_axes", [])
    characterization_axes = truth.get("characterization_translation_axes", [])
    if (not isinstance(required_axes, list) or not isinstance(characterization_axes, list)
            or any(axis not in AXES for axis in required_axes + characterization_axes)):
        raise ValueError("Invalid translation-axis scoring metadata")

    p = prediction if isinstance(prediction, dict) else {}
    status = p.get("status", "missing_prediction")
    completed = status in ("estimated", "no_candidates", "no_component_points")
    candidates = p.get("candidate_count")
    detected = candidates > 0 if completed and isinstance(candidates, int) else None
    component = p.get("detected_component")
    correct = component == truth["injected_component"] if completed and component else None
    pose = p.get("joint_component_pose", {})

    predicted_translation = [None, None, None]
    raw_translation = pose.get("estimated_translation_xyz_mm")
    if isinstance(raw_translation, list) and len(raw_translation) == 3:
        for axis, value in enumerate(raw_translation):
            if isinstance(value, (int, float)) and np.isfinite(value):
                predicted_translation[axis] = float(value)
    predicted_rotation = finite_vector(pose.get("estimated_rotation_xyz_deg"))
    predicted_matrix = pose.get("rotation_matrix_nominal_to_observed")
    valid_rotation = False
    angular_error = None
    rotation_axis_errors = [None, None, None]
    if correct is True and pose.get("status") in ("estimated", "partial_estimate"):
        try:
            matrix = np.asarray(predicted_matrix, dtype=np.float64)
            valid_rotation = (predicted_rotation is not None and matrix.shape == (3, 3)
                              and np.isfinite(matrix).all()
                              and pose.get("quality_status") == "pass_controlled")
            if valid_rotation:
                angular_error = rotation_error_degrees(matrix, expected_rotation_matrix)
                rotation_axis_errors = (predicted_rotation - injected_rotation).tolist()
        except (TypeError, ValueError):
            valid_rotation = False

    translation_errors = [None, None, None]
    valid_axis = [False, False, False]
    for axis in range(3):
        if correct is True and predicted_translation[axis] is not None:
            valid_axis[axis] = True
            translation_errors[axis] = (
                predicted_translation[axis] - float(injected_translation[axis]))
    required_translation_valid = all(
        valid_axis[AXES.index(axis)] for axis in required_axes)
    valid_joint_pose = (category == "detectable_combined_pose" and correct is True
                        and valid_rotation and required_translation_valid)

    any_pose_value = (any(value is not None for value in predicted_translation)
                      or predicted_rotation is not None)
    if not completed:
        outcome = status
    elif category == "zero_defect_control":
        outcome = ("false_positive" if detected or component or any_pose_value
                   else "expected_no_detection")
    elif not detected:
        outcome = "no_detection"
    elif not component:
        outcome = "no_identification"
    elif not correct:
        outcome = "incorrect_component"
    elif not valid_rotation:
        outcome = "missing_rotation_estimate"
    elif not required_translation_valid:
        outcome = "missing_required_translation_axis"
    else:
        outcome = "estimated"

    stages = pose.get("stages") or [{}]
    return {
        "case_id": case_id, "category": category,
        "injected_x_mm": float(injected_translation[0]),
        "injected_y_mm": float(injected_translation[1]),
        "injected_z_mm": float(injected_translation[2]),
        "injected_rx_deg": float(injected_rotation[0]),
        "injected_ry_deg": float(injected_rotation[1]),
        "injected_rz_deg": float(injected_rotation[2]),
        "required_translation_axes": ",".join(required_axes),
        "characterization_translation_axes": ",".join(characterization_axes),
        "prediction_status": status, "outcome": outcome,
        "prediction_completed": completed, "detected": detected,
        "candidate_count": candidates, "detected_component": component,
        "correct_component": correct,
        "predicted_x_mm": predicted_translation[0],
        "predicted_y_mm": predicted_translation[1],
        "predicted_z_mm": predicted_translation[2],
        "signed_error_x_mm": translation_errors[0],
        "signed_error_y_mm": translation_errors[1],
        "signed_error_z_mm": translation_errors[2],
        "predicted_rx_deg": (float(predicted_rotation[0])
                             if valid_rotation else None),
        "predicted_ry_deg": (float(predicted_rotation[1])
                             if valid_rotation else None),
        "predicted_rz_deg": (float(predicted_rotation[2])
                             if valid_rotation else None),
        "signed_error_rx_deg": rotation_axis_errors[0],
        "signed_error_ry_deg": rotation_axis_errors[1],
        "signed_error_rz_deg": rotation_axis_errors[2],
        "angular_error_deg": angular_error,
        "valid_rotation_estimate": valid_rotation,
        "valid_required_translation": required_translation_valid,
        "valid_joint_pose": valid_joint_pose,
        "pose_points_used": pose.get("observed_points_used"),
        "pose_fitness": stages[-1].get("fitness"),
        "pose_rmse_mm": stages[-1].get("inlier_rmse_mm"),
        "surface_p95_mm": p.get("surface_p95_mm"),
        "surface_p99_mm": p.get("surface_p99_mm"),
    }


def error_metrics(values):
    return {
        "valid_estimates": len(values),
        "mean_signed_error": (sum(values) / len(values) if values else None),
        "mean_absolute_error": (
            sum(abs(value) for value in values) / len(values) if values else None),
        "maximum_absolute_error": (
            max(abs(value) for value in values) if values else None),
    }


def aggregate_results(rows):
    detectable = [row for row in rows if row["category"] == "detectable_combined_pose"]
    controls = [row for row in rows if row["category"] == "zero_defect_control"]
    joint = [row for row in detectable if row["valid_joint_pose"]]
    result = {
        "units": "mm", "angle_units": "degrees", "total_cases": len(rows),
        "detection": {
            "expected_detectable_cases": len(detectable),
            "actually_detected_cases": sum(row["detected"] is True for row in detectable),
            "correct_component_identifications": sum(
                row["correct_component"] is True for row in detectable),
            "zero_control_cases": len(controls),
            "zero_control_false_positives": sum(
                row["outcome"] == "false_positive" for row in controls),
            "execution_failures": sum(not row["prediction_completed"] for row in rows),
        },
        "joint_pose": {
            "expected_detectable_cases": len(detectable),
            "valid_joint_pose_estimates": len(joint),
            "excluded_cases": [
                {"case_id": row["case_id"], "reason": row["outcome"]}
                for row in detectable if not row["valid_joint_pose"]],
        },
        "translation": {},
        "rotation": {},
    }
    for axis in AXES:
        field = "signed_error_{}_mm".format(axis)
        expected = [row for row in detectable
                    if axis in (row["required_translation_axes"].split(",")
                                + row["characterization_translation_axes"].split(","))]
        expected = [row for row in expected if axis]
        values = [row[field] for row in expected if row[field] is not None]
        result["translation"][axis] = dict(
            error_metrics(values), expected_axis_cases=len(expected),
            excluded_cases=[row["case_id"] for row in expected if row[field] is None])
    angular = [row["angular_error_deg"] for row in detectable
               if row["valid_rotation_estimate"]]
    result["rotation"] = {
        "expected_rotation_cases": len(detectable),
        "valid_rotation_estimates": len(angular),
        "mean_angular_error_deg": (sum(angular) / len(angular) if angular else None),
        "maximum_angular_error_deg": (max(angular) if angular else None),
        "excluded_cases": [row["case_id"] for row in detectable
                           if not row["valid_rotation_estimate"]],
    }
    for axis in AXES:
        field = "signed_error_r{}_deg".format(axis)
        values = [row[field] for row in detectable if row[field] is not None]
        result["rotation"][axis] = error_metrics(values)
    return result


def run_validation(run_dir, predictor=predict_case, output_dir=None):
    run_dir = Path(run_dir).resolve()
    output_root = run_dir if output_dir is None else Path(output_dir).resolve()
    metadata = read_json(run_dir / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if (metadata.get("experiment") != "combined_component_rigid_pose"
            or metadata.get("generation_status") != "complete"
            or metadata.get("completed_case_ids") != case_ids):
        raise ValueError("Combined-pose generation is incomplete or incompatible")
    if (metadata.get("point_count"), metadata.get("noise_sigma_mm"),
            metadata.get("seed")) != (30000, 0.35, 42):
        raise ValueError("Combined-pose settings do not match the approved sweep")
    reference = run_dir / "reference"
    mesh = reference / "nominal_reference.obj"
    manifest = reference / "component_manifest.json"
    for name, expected in metadata["reference_sha256"].items():
        if file_hash(reference / name) != expected:
            raise ValueError("Frozen reference changed")
    cases = [run_dir / "cases" / case_id for case_id in case_ids]
    if output_dir is not None:
        output_root.mkdir(parents=True, exist_ok=False)

    # Ground truth is deliberately not opened until every prediction is complete.
    for case in cases:
        destination = output_root / "cases" / case.name / "prediction"
        if destination.exists():
            raise FileExistsError(destination)
        predictor(case / "input" / "scan.xyz", mesh, manifest, destination)
    write_json_new(output_root / "prediction_phase.json", {
        "status": "complete", "case_ids": case_ids, "source_run": str(run_dir)})

    rows = []
    for case in cases:
        prediction_path = output_root / "cases" / case.name / "prediction" / "prediction.json"
        try:
            prediction = read_json(prediction_path)
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
    parser = argparse.ArgumentParser(
        description="Evaluate a completed Rhino translation-plus-rotation batch")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(run_validation(args.run_dir, output_dir=args.output_dir),
                     indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
