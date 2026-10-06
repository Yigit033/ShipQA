"""Create immutable, globally misaligned scans from a completed validation run.

This is an external controlled-experiment generator.  The known transform is
written only under each case's ground_truth directory and is never an engine
input.  It does not synthesize new surface noise or alter local defects.
"""

import argparse
import datetime
import json
import shutil
from pathlib import Path

import numpy as np

from registration import apply_transform
from validate_y_translations import file_hash, read_json, write_json_new


def euler_xyz_transform(rx_deg, ry_deg, rz_deg, translation_mm, center_mm):
    """Return CAD-to-scanner transform: Rz*Ry*Rx about center, then translation."""
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    mx = np.array([[1.0, 0.0, 0.0],
                   [0.0, np.cos(rx), -np.sin(rx)],
                   [0.0, np.sin(rx), np.cos(rx)]])
    my = np.array([[np.cos(ry), 0.0, np.sin(ry)],
                   [0.0, 1.0, 0.0],
                   [-np.sin(ry), 0.0, np.cos(ry)]])
    mz = np.array([[np.cos(rz), -np.sin(rz), 0.0],
                   [np.sin(rz), np.cos(rz), 0.0],
                   [0.0, 0.0, 1.0]])
    rotation = mz @ my @ mx
    center = np.asarray(center_mm, dtype=np.float64)
    translation = np.asarray(translation_mm, dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation + center - rotation @ center
    return transform


def generate_run(source_run, output_dir, transform_cad_to_scan):
    source_run = Path(source_run).resolve()
    output_dir = Path(output_dir).resolve()
    metadata = read_json(source_run / "run_metadata.json")
    case_ids = metadata["case_ids"]
    if metadata.get("generation_status") != "complete" or metadata.get("completed_case_ids") != case_ids:
        raise ValueError("Source validation run is incomplete")
    transform = np.asarray(transform_cad_to_scan, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Expected a finite 4x4 CAD-to-scan transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError("Invalid homogeneous transform")

    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "reference").mkdir()
    for name in ("nominal_reference.obj", "component_manifest.json"):
        shutil.copyfile(source_run / "reference" / name, output_dir / "reference" / name)

    completed = []
    for case_id in case_ids:
        source_case = source_run / "cases" / case_id
        case = output_dir / "cases" / case_id
        (case / "input").mkdir(parents=True)
        (case / "ground_truth").mkdir()
        points = np.loadtxt(source_case / "input" / "scan.xyz", dtype=np.float64)
        transformed = apply_transform(points, transform)
        scan_path = case / "input" / "scan.xyz"
        np.savetxt(scan_path, transformed, fmt="%.10f")

        source_truth = read_json(source_case / "ground_truth" / "ground_truth.json")
        truth = dict(source_truth)
        truth.update({
            "schema_version": 1,
            "global_transform_cad_to_scan": transform.tolist(),
            "transform_convention": "column_vector",
            "source_scan_sha256": file_hash(source_case / "input" / "scan.xyz"),
            "scan_sha256": file_hash(scan_path),
        })
        write_json_new(case / "ground_truth" / "ground_truth.json", truth)
        completed.append(case_id)

    reference_hashes = {
        name: file_hash(output_dir / "reference" / name)
        for name in ("nominal_reference.obj", "component_manifest.json")
    }
    write_json_new(output_dir / "run_metadata.json", {
        "schema_version": 1,
        "experiment": "controlled_global_rigid_registration",
        "generation_status": "complete",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "units": "mm",
        "case_ids": case_ids,
        "completed_case_ids": completed,
        "point_count": metadata["point_count"],
        "source_run": str(source_run),
        "source_run_metadata_sha256": file_hash(source_run / "run_metadata.json"),
        "reference_sha256": reference_hashes,
        "generator_sha256": file_hash(Path(__file__)),
        "ground_truth_location": "cases/<neutral_id>/ground_truth/ground_truth.json",
    })
    return output_dir


def main():
    parser = argparse.ArgumentParser(description="Generate a controlled registration validation run")
    parser.add_argument("source_run", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--rotation-deg", type=float, nargs=3, metavar=("RX", "RY", "RZ"),
                        default=(1.2, -0.8, 1.5))
    parser.add_argument("--translation-mm", type=float, nargs=3, metavar=("TX", "TY", "TZ"),
                        default=(75.0, -45.0, 30.0))
    parser.add_argument("--center-mm", type=float, nargs=3, metavar=("CX", "CY", "CZ"),
                        default=(2000.0, 1200.0, 86.0))
    args = parser.parse_args()
    transform = euler_xyz_transform(*args.rotation_deg, args.translation_mm, args.center_mm)
    print(generate_run(args.source_run, args.output_dir, transform))


if __name__ == "__main__":
    main()
