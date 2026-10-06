"""Run geometry-only registered predictions first, then read ground truth."""

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import open3d as o3d

from registration import rotation_angle_degrees
from validate_y_translations import (
    ENGINE,
    aggregate_results,
    evaluate_case,
    file_hash,
    read_json,
    write_json_new,
)


def predict_registered_case(scan, mesh, manifest, output_dir):
    """The prediction subprocess receives geometry and output paths only."""
    completed = subprocess.run(
        [sys.executable, "-B", str(ENGINE), "--scan", str(scan), "--mesh", str(mesh),
         "--manifest", str(manifest), "--output-dir", str(output_dir), "--register"],
        capture_output=True, text=True, check=False,
    )
    if not (output_dir / "prediction.json").exists():
        output_dir.mkdir(parents=True, exist_ok=True)
        with (output_dir / "analysis.log").open("a", encoding="utf-8") as log:
            log.write(completed.stdout + completed.stderr)
        write_json_new(output_dir / "prediction.json", {
            "status": "error", "error": "Engine exited without a prediction",
            "exit_code": completed.returncode,
        })


def registration_errors(prediction, truth, reference_vertices):
    registration = prediction.get("registration") if isinstance(prediction, dict) else None
    if not isinstance(registration, dict) or registration.get("status") != "completed":
        return {"registration_completed": False}
    estimated = np.asarray(registration["transform_scan_to_cad"], dtype=np.float64)
    forward = np.asarray(truth["global_transform_cad_to_scan"], dtype=np.float64)
    if estimated.shape != (4, 4) or forward.shape != (4, 4):
        return {"registration_completed": False}
    expected = np.linalg.inv(forward)
    rotation_delta = estimated[:3, :3] @ expected[:3, :3].T
    round_trip = estimated @ forward
    residual = reference_vertices @ round_trip[:3, :3].T + round_trip[:3, 3] - reference_vertices
    norms = np.linalg.norm(residual, axis=1)
    return {
        "registration_completed": True,
        "registration_rotation_error_deg": rotation_angle_degrees(rotation_delta),
        "registration_translation_parameter_error_mm": float(
            np.linalg.norm(estimated[:3, 3] - expected[:3, 3])),
        "registration_reference_rmse_mm": float(np.sqrt(np.mean(norms * norms))),
        "registration_reference_max_error_mm": float(norms.max()),
        "registration_pre_p95_mm": registration.get("pre_surface_p95_mm"),
        "registration_post_p95_mm": registration.get("post_surface_p95_mm"),
        "registration_fine_fitness": registration.get("stages", [{}])[-1].get("fitness"),
    }


def run_validation(run_dir, output_dir, predictor=predict_registered_case):
    run_dir = Path(run_dir).resolve()
    output_dir = Path(output_dir).resolve()
    metadata = read_json(run_dir / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if (metadata.get("experiment") != "controlled_global_rigid_registration"
            or metadata.get("generation_status") != "complete"
            or metadata.get("completed_case_ids") != case_ids
            or len(set(case_ids)) != len(case_ids)):
        raise ValueError("Registration run is incomplete or invalid")
    mesh_path = run_dir / "reference" / "nominal_reference.obj"
    manifest_path = run_dir / "reference" / "component_manifest.json"
    for name, expected in metadata["reference_sha256"].items():
        if file_hash(run_dir / "reference" / name) != expected:
            raise ValueError("Frozen reference has changed")
    for case_id in case_ids:
        if not (run_dir / "cases" / case_id / "input" / "scan.xyz").is_file():
            raise FileNotFoundError(case_id)
    output_dir.mkdir(parents=True, exist_ok=False)

    # Prediction phase: no ground-truth path or value crosses this boundary.
    for case_id in case_ids:
        case = run_dir / "cases" / case_id
        predictor(case / "input" / "scan.xyz", mesh_path, manifest_path,
                  output_dir / "cases" / case_id / "prediction")
    write_json_new(output_dir / "prediction_phase.json", {
        "status": "complete", "case_ids": case_ids, "source_run": str(run_dir),
        "engine_sha256": file_hash(ENGINE),
        "registration_kernel_sha256": file_hash(ENGINE.with_name("registration.py")),
        "distance_kernel_sha256": file_hash(ENGINE.with_name("surface_distance.py")),
        "evaluator_sha256": file_hash(Path(__file__)),
    })

    # Evaluation phase starts only after every prediction invocation returns.
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    reference_vertices = np.asarray(mesh.vertices, dtype=np.float64)
    rows = []
    for case_id in case_ids:
        case = run_dir / "cases" / case_id
        prediction_path = output_dir / "cases" / case_id / "prediction" / "prediction.json"
        prediction = read_json(prediction_path) if prediction_path.is_file() else None
        truth = read_json(case / "ground_truth" / "ground_truth.json")
        scan_hash = file_hash(case / "input" / "scan.xyz")
        if truth.get("scan_sha256") != scan_hash:
            raise ValueError("Ground truth is not bound to scan: " + case_id)
        if prediction and prediction.get("status") != "error":
            expected_hashes = {"scan": scan_hash, "mesh": file_hash(mesh_path),
                               "manifest": file_hash(manifest_path)}
            if prediction.get("input_sha256") != expected_hashes:
                prediction = dict(prediction, status="input_mismatch")
        row = evaluate_case(case_id, prediction, truth)
        row.update(registration_errors(prediction or {}, truth, reference_vertices))
        rows.append(row)

    with (output_dir / "summary.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    completed = [r for r in rows if r["registration_completed"]]
    local = aggregate_results(rows)
    aggregate = {
        "schema_version": 1,
        "units": "mm",
        "registration": {
            "expected_cases": len(rows),
            "completed_cases": len(completed),
            "failed_cases": [r["case_id"] for r in rows if not r["registration_completed"]],
            "mean_rotation_error_deg": (
                sum(r["registration_rotation_error_deg"] for r in completed) / len(completed)
                if completed else None),
            "maximum_rotation_error_deg": (
                max(r["registration_rotation_error_deg"] for r in completed)
                if completed else None),
            "mean_reference_rmse_mm": (
                sum(r["registration_reference_rmse_mm"] for r in completed) / len(completed)
                if completed else None),
            "maximum_reference_error_mm": (
                max(r["registration_reference_max_error_mm"] for r in completed)
                if completed else None),
        },
        "downstream_qa": local,
    }
    if any(not math.isfinite(value) for row in completed for value in (
            row["registration_rotation_error_deg"], row["registration_reference_rmse_mm"])):
        raise ValueError("Non-finite registration metric")
    write_json_new(output_dir / "aggregate.json", aggregate)
    return aggregate


def main():
    parser = argparse.ArgumentParser(description="Validate global registration before local QA")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(run_validation(args.run_dir, args.output_dir), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
