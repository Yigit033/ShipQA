"""Predict the complete robustness matrix before evaluating any ground truth."""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import open3d as o3d

from validate_registration import registration_errors
from validate_y_translations import (
    ENGINE, aggregate_results, evaluate_case, file_hash, read_json, write_json_new)


ACCEPTED_QA_STATUSES = ("estimated", "no_candidates", "no_component_points")


def predict_case(case, mesh, manifest, output_dir):
    """Read prediction configuration only; ground truth is a separate phase."""
    request_path = case / "input" / "analysis_request.json"
    request = read_json(request_path)
    if request.get("registration") is not True:
        raise ValueError("Robustness cases must request registration")
    command = [
        sys.executable, "-B", str(ENGINE),
        "--scan", str(case / "input" / "scan.xyz"),
        "--mesh", str(mesh), "--manifest", str(manifest),
        "--output-dir", str(output_dir), "--register",
    ]
    alignment_name = request.get("initial_alignment")
    if alignment_name is not None:
        if alignment_name != "initial_alignment.json":
            raise ValueError("Unexpected initial-alignment filename")
        command.extend(["--initial-alignment", str(case / "input" / alignment_name)])
    if request.get("filter_isolated_outliers") is True:
        command.append("--filter-isolated-outliers")
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if not (output_dir / "prediction.json").exists():
        output_dir.mkdir(parents=True, exist_ok=True)
        with (output_dir / "analysis.log").open("a", encoding="utf-8") as log:
            log.write(completed.stdout + completed.stderr)
        write_json_new(output_dir / "prediction.json", {
            "status": "error", "error": "Engine exited without a prediction",
            "exit_code": completed.returncode,
        })


def _rejected_row(case_id, prediction, truth):
    registration = prediction.get("registration", {}) if isinstance(prediction, dict) else {}
    return {
        "case_id": case_id,
        "category": truth["category"],
        "injected_component": truth["injected_component"],
        "injected_shift_y_mm": float(truth["injected_shift_y_mm"]),
        "prediction_status": prediction.get("status", "missing_prediction"),
        "outcome": "qa_blocked_by_registration",
        "prediction_completed": False,
        "detected": None,
        "candidate_count": prediction.get("candidate_count"),
        "detected_component": prediction.get("detected_component"),
        "correct_component": None,
        "false_positive_component": None,
        "predicted_shift_y_mm": prediction.get("estimated_shift_y_mm"),
        "points_used": prediction.get("points_used"),
        "surface_p95_mm": prediction.get("surface_p95_mm"),
        "surface_p99_mm": prediction.get("surface_p99_mm"),
        "signed_error_mm": None,
        "absolute_error_mm": None,
        "relative_absolute_error_percent": None,
        "valid_estimate": False,
        "eligible_for_estimation_metrics": False,
        "registration_quality_status": registration.get("quality_status"),
    }


def run_validation(run_dir, output_dir, predictor=predict_case):
    run_dir = Path(run_dir).resolve()
    output_dir = Path(output_dir).resolve()
    metadata = read_json(run_dir / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if (metadata.get("experiment") != "registration_robustness_matrix"
            or metadata.get("generation_status") != "complete"
            or metadata.get("completed_case_ids") != case_ids
            or len(set(case_ids)) != len(case_ids)):
        raise ValueError("Robustness run is incomplete or invalid")
    mesh_path = run_dir / "reference" / "nominal_reference.obj"
    manifest_path = run_dir / "reference" / "component_manifest.json"
    for name, expected in metadata["reference_sha256"].items():
        if file_hash(run_dir / "reference" / name) != expected:
            raise ValueError("Frozen reference has changed")
    cases = [run_dir / "cases" / case_id for case_id in case_ids]
    for case in cases:
        for name in ("scan.xyz", "analysis_request.json"):
            if not (case / "input" / name).is_file():
                raise FileNotFoundError(case / "input" / name)
    output_dir.mkdir(parents=True, exist_ok=False)

    # No ground-truth reads are allowed before every engine call returns.
    for case in cases:
        predictor(case, mesh_path, manifest_path,
                  output_dir / "cases" / case.name / "prediction")
    write_json_new(output_dir / "prediction_phase.json", {
        "status": "complete", "case_ids": case_ids, "source_run": str(run_dir),
        "engine_sha256": file_hash(ENGINE),
        "registration_kernel_sha256": file_hash(ENGINE.with_name("registration.py")),
        "distance_kernel_sha256": file_hash(ENGINE.with_name("surface_distance.py")),
        "evaluator_sha256": file_hash(Path(__file__)),
    })

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    reference_vertices = np.asarray(mesh.vertices, dtype=np.float64)
    rows = []
    for case in cases:
        prediction_path = output_dir / "cases" / case.name / "prediction" / "prediction.json"
        prediction = read_json(prediction_path) if prediction_path.is_file() else {}
        truth = read_json(case / "ground_truth" / "ground_truth.json")
        scan_path = case / "input" / "scan.xyz"
        if truth.get("scan_sha256") != file_hash(scan_path):
            raise ValueError("Ground truth is not bound to scan: " + case.name)
        request = read_json(case / "input" / "analysis_request.json")
        expected_hashes = {
            "scan": file_hash(scan_path), "mesh": file_hash(mesh_path),
            "manifest": file_hash(manifest_path),
        }
        if request.get("initial_alignment"):
            expected_hashes["initial_alignment"] = file_hash(
                case / "input" / request["initial_alignment"])
        if prediction.get("status") != "error" and prediction.get("input_sha256") != expected_hashes:
            prediction = dict(prediction, status="input_mismatch")

        accepted = prediction.get("status") in ACCEPTED_QA_STATUSES
        rejected = prediction.get("status") == "registration_rejected"
        expected_accept = bool(truth["expected_registration_accepted"])
        if accepted:
            row = evaluate_case(case.name, prediction, truth)
            row["registration_quality_status"] = prediction["registration"].get("quality_status")
        else:
            row = _rejected_row(case.name, prediction, truth)
        row.update({
            "scenario": truth["scenario"],
            "expected_registration_accepted": expected_accept,
            "registration_accepted": accepted,
            "registration_rejected": rejected,
            "acceptance_behavior_correct": accepted == expected_accept,
            "input_point_count": prediction.get("scan_point_count"),
            "analysis_point_count": prediction.get("analysis_point_count"),
        })
        row.update(registration_errors(prediction, truth, reference_vertices))
        rows.append(row)

    with (output_dir / "summary.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    scenario_reports = {}
    for scenario in sorted({row["scenario"] for row in rows}):
        group = [row for row in rows if row["scenario"] == scenario]
        accepted_group = [row for row in group if row["registration_accepted"]]
        scenario_reports[scenario] = {
            "cases": len(group),
            "expected_acceptance": group[0]["expected_registration_accepted"],
            "accepted": sum(row["registration_accepted"] for row in group),
            "rejected": sum(row["registration_rejected"] for row in group),
            "acceptance_behavior_correct": sum(row["acceptance_behavior_correct"] for row in group),
            "mean_reference_rmse_mm": (
                sum(row["registration_reference_rmse_mm"] for row in group
                    if row.get("registration_reference_rmse_mm") is not None) / len(group)),
            "downstream_qa": aggregate_results(accepted_group) if accepted_group else None,
        }
    aggregate = {
        "schema_version": 1, "units": "mm", "total_cases": len(rows),
        "registration_acceptance": {
            "correct_cases": sum(row["acceptance_behavior_correct"] for row in rows),
            "expected_cases": len(rows),
            "unexpected_acceptances": [row["case_id"] for row in rows
                                       if row["registration_accepted"]
                                       and not row["expected_registration_accepted"]],
            "unexpected_rejections": [row["case_id"] for row in rows
                                      if not row["registration_accepted"]
                                      and row["expected_registration_accepted"]],
        },
        "scenarios": scenario_reports,
    }
    write_json_new(output_dir / "aggregate.json", aggregate)
    return aggregate


def main():
    parser = argparse.ArgumentParser(description="Validate registration robustness matrix")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(run_validation(args.run_dir, args.output_dir), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
