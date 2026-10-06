"""Generate a focused registration robustness matrix from frozen Rhino scans."""

import argparse
import datetime
import shutil
from pathlib import Path

import numpy as np

from generate_registration_validation import euler_xyz_transform
from registration import apply_transform
from validate_y_translations import file_hash, read_json, write_json_new


PANEL_CENTER_MM = [2000.0, 1200.0, 86.0]
SOURCE_SHIFTS_MM = (0.0, -8.0, 12.0)

# A small matrix with a reason for every case.  More combinations would add run
# time without answering a new engineering question at this stage.
SCENARIOS = (
    {"name": "small_auto", "rotation_deg": [0.4, -0.3, 0.6],
     "translation_mm": [25.0, -35.0, 15.0], "expected_accept": True},
    {"name": "opposite_auto", "rotation_deg": [-1.5, 1.0, -2.0],
     "translation_mm": [-120.0, 80.0, -50.0], "expected_accept": True},
    {"name": "large_auto", "rotation_deg": [3.0, -2.5, 4.0],
     "translation_mm": [250.0, -180.0, 120.0], "expected_accept": True},
    {"name": "partial_with_correspondences", "rotation_deg": [2.0, -1.5, 2.5],
     "translation_mm": [180.0, -130.0, 80.0], "partial_x_mm": [600.0, 3400.0],
     "initial_alignment": "corresponding_points", "expected_accept": True},
    {"name": "isolated_outliers_filtered", "rotation_deg": [1.2, -0.8, 1.5],
     "translation_mm": [75.0, -45.0, 30.0], "outlier_fraction": 0.10,
     "filter_isolated_outliers": True, "expected_accept": True},
    {"name": "arbitrary_pose_with_scanner_transform", "rotation_deg": [15.0, -10.0, 20.0],
     "translation_mm": [500.0, -350.0, 240.0], "initial_alignment": "transform_matrix",
     "expected_accept": True},
    {"name": "partial_without_control", "rotation_deg": [2.0, -1.5, 2.5],
     "translation_mm": [180.0, -130.0, 80.0], "partial_x_mm": [0.0, 1400.0],
     "expected_accept": False},
    {"name": "excessive_rotation_without_control", "rotation_deg": [18.0, -12.0, 20.0],
     "translation_mm": [500.0, -350.0, 240.0], "expected_accept": False},
)


def _select_source_cases(source_run):
    selected = {}
    metadata = read_json(source_run / "run_metadata.json")
    for case_id in metadata["case_ids"]:
        case = source_run / "cases" / case_id
        truth = read_json(case / "ground_truth" / "ground_truth.json")
        shift = float(truth["injected_shift_y_mm"])
        if shift in SOURCE_SHIFTS_MM:
            if shift in selected:
                raise ValueError("Duplicate source shift: {}".format(shift))
            selected[shift] = (case, truth)
    if set(selected) != set(SOURCE_SHIFTS_MM):
        raise ValueError("Source run does not contain the required control shifts")
    return selected


def _control_points():
    return np.array([
        [0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], [0.0, 2400.0, 0.0],
        [4000.0, 2400.0, 0.0], [0.0, 1200.0, 172.0],
        [4000.0, 1200.0, 172.0],
    ])


def _write_initial_alignment(path, method, forward, seed):
    if method == "corresponding_points":
        cad_points = _control_points()
        scan_points = apply_transform(cad_points, forward)
        rng = np.random.default_rng(seed)
        # Controlled total-station/manual picking uncertainty, not exact ground truth.
        scan_points += rng.normal(0.0, 0.35, size=scan_points.shape)
        value = {
            "schema_version": 1, "units": "mm", "method": method,
            "source": "synthetic_survey_or_engineer_selected_targets",
            "scan_points_mm": scan_points.tolist(), "cad_points_mm": cad_points.tolist(),
        }
    elif method == "transform_matrix":
        error = euler_xyz_transform(
            0.05, -0.08, 0.10, [1.0, -1.0, 0.5], PANEL_CENTER_MM)
        value = {
            "schema_version": 1, "units": "mm", "method": method,
            "source": "synthetic_scanner_pose_or_survey_transform",
            "transform_scan_to_cad": (error @ np.linalg.inv(forward)).tolist(),
        }
    else:
        raise ValueError("Unknown initial alignment method")
    write_json_new(path, value)


def _add_isolated_outliers(points, fraction, seed):
    count = int(round(len(points) * fraction))
    rng = np.random.default_rng(seed)
    low = points.min(axis=0) + [-1000.0, -1000.0, -600.0]
    high = points.max(axis=0) + [1000.0, 1000.0, 800.0]
    outliers = rng.uniform(low, high, size=(count, 3))
    return np.vstack([points, outliers]), count


def generate_run(source_run, output_dir):
    source_run = Path(source_run).resolve()
    output_dir = Path(output_dir).resolve()
    source_metadata = read_json(source_run / "run_metadata.json")
    if source_metadata.get("generation_status") != "complete":
        raise ValueError("Source run is incomplete")
    sources = _select_source_cases(source_run)

    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "reference").mkdir()
    for name in ("nominal_reference.obj", "component_manifest.json"):
        shutil.copyfile(source_run / "reference" / name, output_dir / "reference" / name)

    case_ids = []
    for scenario_index, scenario in enumerate(SCENARIOS):
        forward = euler_xyz_transform(
            *scenario["rotation_deg"], scenario["translation_mm"], PANEL_CENTER_MM)
        for shift_index, shift in enumerate(SOURCE_SHIFTS_MM):
            source_case, source_truth = sources[shift]
            case_id = "case_{:03d}".format(len(case_ids) + 1)
            case_ids.append(case_id)
            case = output_dir / "cases" / case_id
            (case / "input").mkdir(parents=True)
            (case / "ground_truth").mkdir()

            points = np.loadtxt(source_case / "input" / "scan.xyz", dtype=np.float64)
            if "partial_x_mm" in scenario:
                low, high = scenario["partial_x_mm"]
                points = points[(points[:, 0] >= low) & (points[:, 0] <= high)]
            points = apply_transform(points, forward)
            outlier_count = 0
            if scenario.get("outlier_fraction"):
                points, outlier_count = _add_isolated_outliers(
                    points, scenario["outlier_fraction"], 1000 + scenario_index * 10 + shift_index)
            scan_path = case / "input" / "scan.xyz"
            np.savetxt(scan_path, points, fmt="%.10f")

            alignment_name = None
            if scenario.get("initial_alignment"):
                alignment_name = "initial_alignment.json"
                _write_initial_alignment(
                    case / "input" / alignment_name, scenario["initial_alignment"],
                    forward, 2000 + scenario_index * 10 + shift_index)
            write_json_new(case / "input" / "analysis_request.json", {
                "schema_version": 1,
                "registration": True,
                "filter_isolated_outliers": bool(
                    scenario.get("filter_isolated_outliers", False)),
                "initial_alignment": alignment_name,
            })

            truth = dict(source_truth)
            truth.update({
                "schema_version": 1,
                "scenario": scenario["name"],
                "expected_registration_accepted": scenario["expected_accept"],
                "global_transform_cad_to_scan": forward.tolist(),
                "transform_convention": "column_vector",
                "partial_x_mm": scenario.get("partial_x_mm"),
                "injected_outlier_count": outlier_count,
                "source_scan_sha256": file_hash(source_case / "input" / "scan.xyz"),
                "scan_sha256": file_hash(scan_path),
            })
            write_json_new(case / "ground_truth" / "ground_truth.json", truth)

    write_json_new(output_dir / "run_metadata.json", {
        "schema_version": 1,
        "experiment": "registration_robustness_matrix",
        "generation_status": "complete",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "units": "mm",
        "case_ids": case_ids,
        "completed_case_ids": case_ids,
        "scenario_count": len(SCENARIOS),
        "source_shifts_mm": list(SOURCE_SHIFTS_MM),
        "source_run": str(source_run),
        "source_run_metadata_sha256": file_hash(source_run / "run_metadata.json"),
        "reference_sha256": {
            name: file_hash(output_dir / "reference" / name)
            for name in ("nominal_reference.obj", "component_manifest.json")
        },
        "generator_sha256": file_hash(Path(__file__)),
    })
    return output_dir


def main():
    parser = argparse.ArgumentParser(description="Generate focused registration robustness cases")
    parser.add_argument("source_run", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    print(generate_run(args.source_run, args.output_dir))


if __name__ == "__main__":
    main()
