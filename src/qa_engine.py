import argparse
import contextlib
import hashlib
import json
import platform
import traceback
from pathlib import Path

import numpy as np
import open3d as o3d

from registration import initial_transform_from_spec, register_scan_to_nominal
from surface_distance import compute_surface_deviation
from component_pose import estimate_translation_xyz


# ---------------------------------------------------------
# PATHS
# ---------------------------------------------------------

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "data"

DEFECT_THRESHOLD_MM = 5.0
DEFECT_CLUSTER_RADIUS_MM = 100.0
MIN_DEFECT_CLUSTER_POINTS = 10
MIN_REGISTRATION_FINE_FITNESS = 0.85
MAX_REGISTRATION_SURFACE_P95_MM = 2.0
MIN_REGISTRATION_RETAINED_FRACTION = 0.75
MIN_AUTOMATIC_COVERAGE_RATIO_XYZ = np.array([0.60, 0.60, 0.50])

SCAN_PATH = DATA_DIR / "synthetic_scan_v1.xyz"
MESH_PATH = DATA_DIR / "nominal_reference.obj"
MANIFEST_PATH = DATA_DIR / "component_manifest.json"

def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def analyze_scan(scan_path=SCAN_PATH, mesh_path=MESH_PATH, manifest_path=MANIFEST_PATH,
                 defect_output=DATA_DIR / "defect_candidates.xyz", registration=False,
                 registered_scan_output=None, initial_alignment_path=None,
                 filter_isolated_outliers=False):
    """Predict from measured coordinates and nominal geometry only; all lengths in mm."""
    scan_path, mesh_path, manifest_path = map(Path, (scan_path, mesh_path, manifest_path))
    defect_output = Path(defect_output)
    # ---------------------------------------------------------
    # CHECK FILES
    # ---------------------------------------------------------

    if not scan_path.exists():
        raise FileNotFoundError(f"Scan not found: {scan_path}")

    if not mesh_path.exists():
        raise FileNotFoundError(f"Nominal mesh not found: {mesh_path}")


    # ---------------------------------------------------------
    # LOAD SYNTHETIC SCAN
    # ---------------------------------------------------------

    scan_points = np.loadtxt(scan_path, dtype=np.float64)

    if scan_points.ndim != 2 or scan_points.shape[1] != 3:
        raise ValueError(
            f"Expected XYZ scan with shape (N, 3), got {scan_points.shape}"
        )


    if len(scan_points) == 0 or not np.isfinite(scan_points).all():
        raise ValueError("Scan must contain finite XYZ points.")

    # ---------------------------------------------------------
    # LOAD NOMINAL CAD MESH
    # ---------------------------------------------------------

    nominal_mesh = o3d.io.read_triangle_mesh(str(mesh_path))

    if nominal_mesh.is_empty() or not nominal_mesh.has_triangles():
        raise RuntimeError("Nominal mesh could not be loaded.")

    vertices = np.asarray(nominal_mesh.vertices)
    triangles = np.asarray(nominal_mesh.triangles)


    # ---------------------------------------------------------
    # BASIC QA
    # ---------------------------------------------------------

    raw_scan_min = scan_points.min(axis=0)
    raw_scan_max = scan_points.max(axis=0)
    raw_scan_point_count = len(scan_points)

    mesh_min = vertices.min(axis=0)
    mesh_max = vertices.max(axis=0)

    registration_result = {"status": "not_requested"}
    if registration:
        initial_transform = None
        initial_details = None
        if initial_alignment_path is not None:
            initial_alignment_path = Path(initial_alignment_path)
            with initial_alignment_path.open(encoding="utf-8") as stream:
                initial_spec = json.load(stream)
            initial_transform, initial_details = initial_transform_from_spec(initial_spec)
        pre_registration_distances = compute_surface_deviation(
            scan_points, vertices, triangles)
        scan_points, retained_indices, registration_result = register_scan_to_nominal(
            scan_points, vertices, triangles, initial_transform=initial_transform,
            filter_isolated_outliers=filter_isolated_outliers)
        registration_result["coarse_alignment"] = initial_details
        registration_result["pre_surface_median_mm"] = float(
            np.median(pre_registration_distances))
        registration_result["pre_surface_p95_mm"] = float(
            np.percentile(pre_registration_distances, 95))
        registration_result["pre_surface_p99_mm"] = float(
            np.percentile(pre_registration_distances, 99))
        if registered_scan_output is not None:
            np.savetxt(registered_scan_output, scan_points, fmt="%.10f")
    elif initial_alignment_path is not None or filter_isolated_outliers:
        raise ValueError("Initial alignment and outlier filtering require registration")

    scan_min = scan_points.min(axis=0)
    scan_max = scan_points.max(axis=0)


    print("\n========================================")
    print("SHIPQA - DATA VALIDATION")
    print("========================================")

    print(f"Scan points       : {len(scan_points):,}")
    print(f"Mesh vertices     : {len(vertices):,}")
    print(f"Mesh triangles    : {len(triangles):,}")

    if registration:
        print("\nRAW SCAN BOUNDS [mm]")
        print(f"Min XYZ: {raw_scan_min}")
        print(f"Max XYZ: {raw_scan_max}")
        print("\nREGISTERED SCAN BOUNDS [mm]")
    else:
        print("\nSCAN BOUNDS [mm]")
    print(f"Min XYZ: {scan_min}")
    print(f"Max XYZ: {scan_max}")

    print("\nNOMINAL CAD BOUNDS [mm]")
    print(f"Min XYZ: {mesh_min}")
    print(f"Max XYZ: {mesh_max}")

    print("\nData loading completed successfully.")
    print("========================================\n")

    # ---------------------------------------------------------
    # POINT-TO-MESH SURFACE DEVIATION
    # ---------------------------------------------------------

    # Float64 face/edge/vertex distances avoid float32 barycentric cancellation
    # on the long, thin triangles in the exported panel. Units remain mm.
    distances = compute_surface_deviation(scan_points, vertices, triangles)

    if registration:
        registration_result["post_surface_median_mm"] = float(np.median(distances))
        registration_result["post_surface_p95_mm"] = float(np.percentile(distances, 95))
        registration_result["post_surface_p99_mm"] = float(np.percentile(distances, 99))
        pre_p95 = registration_result["pre_surface_p95_mm"]
        registration_result["surface_p95_improvement_ratio"] = (
            None if pre_p95 == 0.0 else
            float(registration_result["post_surface_p95_mm"] / pre_p95)
        )
        fine_fitness = registration_result["stages"][-1]["fitness"]
        mesh_span = mesh_max - mesh_min
        scan_span = scan_max - scan_min
        coverage_ratio = np.divide(
            scan_span, mesh_span, out=np.full(3, np.inf), where=mesh_span > 0.0)
        registration_result["registered_span_to_nominal_ratio_xyz"] = coverage_ratio.tolist()
        has_external_coarse_alignment = initial_alignment_path is not None
        registration_result["quality_checks"] = {
            "fine_fitness_at_least_0_85": bool(
                fine_fitness >= MIN_REGISTRATION_FINE_FITNESS),
            "post_surface_p95_at_most_2_mm": bool(
                registration_result["post_surface_p95_mm"]
                <= MAX_REGISTRATION_SURFACE_P95_MM),
            "retained_point_fraction_at_least_0_75": bool(
                registration_result["retained_point_fraction"]
                >= MIN_REGISTRATION_RETAINED_FRACTION),
            "automatic_alignment_has_sufficient_xyz_coverage": bool(
                has_external_coarse_alignment
                or np.all(coverage_ratio >= MIN_AUTOMATIC_COVERAGE_RATIO_XYZ)),
        }
        registration_result["quality_status"] = (
            "pass_provisional" if all(registration_result["quality_checks"].values())
            else "fail"
        )
        registration_result["quality_scope"] = (
            "Controlled scans; thresholds require project-specific field qualification"
        )
        print("\n========================================")
        print("SHIPQA - GLOBAL RIGID REGISTRATION")
        print("========================================")
        print("Transform maps raw scan -> nominal CAD coordinates")
        print(np.asarray(registration_result["transform_scan_to_cad"]))
        print("Pre-registration P95 : {:.3f} mm".format(pre_p95))
        print("Post-registration P95: {:.3f} mm".format(
            registration_result["post_surface_p95_mm"]))
        print("Fine ICP fitness     : {:.6f}".format(fine_fitness))
        print("Quality status       : {}".format(registration_result["quality_status"]))
        print("========================================\n")


    # ---------------------------------------------------------
    # BASIC DEVIATION STATISTICS
    # ---------------------------------------------------------

    print("\n========================================")
    print("SHIPQA - SURFACE DEVIATION")
    print("========================================")

    print(f"Minimum deviation : {distances.min():.3f} mm")
    print(f"Median deviation  : {np.median(distances):.3f} mm")
    print(f"Mean deviation    : {distances.mean():.3f} mm")

    print(f"P95 deviation     : {np.percentile(distances, 95):.3f} mm")
    print(f"P99 deviation     : {np.percentile(distances, 99):.3f} mm")

    print(f"Maximum deviation : {distances.max():.3f} mm")

    print("\nTHRESHOLD COUNTS")

    for threshold in [1.0, 2.0, 5.0, 10.0]:
        count = np.sum(distances > threshold)
        percentage = (count / len(distances)) * 100.0

        print(
            f"> {threshold:4.1f} mm : "
            f"{count:6,d} points "
            f"({percentage:6.2f}%)"
        )

    print("========================================\n")

    # ---------------------------------------------------------
    # LOCALIZE LARGE DEVIATIONS
    # ---------------------------------------------------------

    defect_mask = distances > DEFECT_THRESHOLD_MM
    raw_defect_points = scan_points[defect_mask]
    raw_defect_distances = distances[defect_mask]
    if len(raw_defect_points) > 0:
        candidate_cloud = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(raw_defect_points))
        cluster_labels = np.asarray(candidate_cloud.cluster_dbscan(
            eps=DEFECT_CLUSTER_RADIUS_MM,
            min_points=MIN_DEFECT_CLUSTER_POINTS,
            print_progress=False,
        ))
        supported_mask = cluster_labels >= 0
        defect_points = raw_defect_points[supported_mask]
        defect_distances = raw_defect_distances[supported_mask]
        defect_cluster_count = len(set(cluster_labels[supported_mask].tolist()))
    else:
        defect_points = raw_defect_points
        defect_distances = raw_defect_distances
        defect_cluster_count = 0

    result = {
        "schema_version": 1, "status": "no_candidates", "units": "mm",
        "scan_point_count": raw_scan_point_count, "analysis_point_count": len(scan_points),
        "mesh_vertex_count": len(vertices),
        "mesh_triangle_count": len(triangles), "raw_scan_min_mm": raw_scan_min.tolist(),
        "raw_scan_max_mm": raw_scan_max.tolist(), "scan_min_mm": scan_min.tolist(),
        "scan_max_mm": scan_max.tolist(), "mesh_min_mm": mesh_min.tolist(),
        "mesh_max_mm": mesh_max.tolist(), "defect_threshold_mm": DEFECT_THRESHOLD_MM,
        "raw_candidate_count": len(raw_defect_points),
        "candidate_count": len(defect_points),
        "rejected_sparse_candidate_count": len(raw_defect_points) - len(defect_points),
        "defect_cluster_radius_mm": DEFECT_CLUSTER_RADIUS_MM,
        "minimum_defect_cluster_points": MIN_DEFECT_CLUSTER_POINTS,
        "defect_cluster_count": defect_cluster_count,
        "detected_component": None,
        "points_used": 0, "nominal_center_y_mm": None, "observed_center_y_mm": None,
        "estimated_shift_y_mm": None, "estimated_translation_xyz_mm": None,
        "component_pose": {"status": "not_estimated"},
        "surface_p95_mm": float(np.percentile(distances, 95)),
        "surface_p99_mm": float(np.percentile(distances, 99)),
        "surface_min_mm": float(distances.min()), "surface_median_mm": float(np.median(distances)),
        "surface_mean_mm": float(distances.mean()), "surface_max_mm": float(distances.max()),
        "input_sha256": {"scan": file_hash(scan_path), "mesh": file_hash(mesh_path),
                         "manifest": file_hash(manifest_path)},
        "engine_sha256": file_hash(Path(__file__)),
        "distance_method": "float64_exhaustive_triangle_surface",
        "distance_kernel_sha256": file_hash(Path(__file__).with_name("surface_distance.py")),
        "component_pose_kernel_sha256": file_hash(
            Path(__file__).with_name("component_pose.py")),
        "registration": registration_result,
        "registration_kernel_sha256": (
            file_hash(Path(__file__).with_name("registration.py")) if registration else None),
        "versions": {"python": platform.python_version(), "numpy": np.__version__,
                     "open3d": o3d.__version__},
    }
    if initial_alignment_path is not None:
        result["input_sha256"]["initial_alignment"] = file_hash(initial_alignment_path)

    if registration and registration_result["quality_status"] == "fail":
        result.update(status="registration_rejected", candidate_count=None)
        np.savetxt(defect_output, np.empty((0, 3)), fmt="%.4f")
        print("Registration quality gate rejected this scan; component QA was not run.")
        return result

    print("\n========================================")
    print("SHIPQA - DEFECT LOCALIZATION")
    print("========================================")

    print(f"Threshold          : {DEFECT_THRESHOLD_MM:.1f} mm")
    print(f"Raw threshold points: {len(raw_defect_points):,}")
    print(f"Supported candidates: {len(defect_points):,}")
    print(f"Candidate clusters : {defect_cluster_count:,}")

    if len(defect_points) > 0:

        defect_min = defect_points.min(axis=0)
        defect_max = defect_points.max(axis=0)
        defect_center = defect_points.mean(axis=0)

        worst_index = np.argmax(defect_distances)

        worst_point = defect_points[worst_index]
        worst_distance = defect_distances[worst_index]

        print("\nDEFECT REGION BOUNDS [mm]")
        print(f"Min XYZ : {defect_min}")
        print(f"Max XYZ : {defect_max}")

        print("\nDEFECT REGION CENTER [mm]")
        print(defect_center)

        print("\nWORST POINT")
        print(f"XYZ      : {worst_point}")
        print(f"Deviation: {worst_distance:.3f} mm")

        # Export candidates so we can inspect them in Rhino later

        np.savetxt(
            defect_output,
            defect_points,
            fmt="%.4f"
        )

        print(f"\nDefect candidates exported:")
        print(defect_output)

    print("========================================\n")

    # ---------------------------------------------------------
    # MAP DEFECT REGION TO ENGINEERING COMPONENT
    # ---------------------------------------------------------

    if not manifest_path.exists():
        raise FileNotFoundError(
            f"Component manifest not found: {manifest_path}"
        )

    with open(manifest_path, "r") as f:
        manifest = json.load(f)

    if manifest.get("units") != "mm":
        raise ValueError("Component manifest units must be mm.")
    components = manifest["components"]
    if not components:
        raise ValueError("Component manifest is empty.")

    if len(defect_points) == 0:
        # A detector outcome, not a fabricated zero displacement or a runtime failure.
        np.savetxt(defect_output, defect_points, fmt="%.4f")
        print("No defect candidates; no component displacement estimate.")
        return result

    defect_center_y = defect_points[:, 1].mean()

    closest_component = None
    closest_distance = None

    for component in components:

        nominal_y = component["nominal_center"][1]

        distance_y = abs(defect_center_y - nominal_y)

        if closest_distance is None or distance_y < closest_distance:

            closest_distance = distance_y
            closest_component = component


    print("\n========================================")
    print("SHIPQA - COMPONENT IDENTIFICATION")
    print("========================================")

    print(f"Defect center Y     : {defect_center_y:.3f} mm")

    if closest_component:

        assembly_id = closest_component["assembly_id"]
        result["detected_component"] = assembly_id
        result["status"] = "no_component_points"
        result["defect_center_mm"] = defect_center.tolist()
        result["defect_min_mm"] = defect_min.tolist()
        result["defect_max_mm"] = defect_max.tolist()
        nominal_y = closest_component["nominal_center"][1]

        print(f"Detected component  : {assembly_id}")
        print(f"Nominal center Y    : {nominal_y:.3f} mm")
        print(
            f"Distance to nominal : "
            f"{abs(defect_center_y - nominal_y):.3f} mm"
        )

    print("========================================\n")

    # ---------------------------------------------------------
    # ESTIMATE COMPONENT TRANSVERSE SHIFT
    # ---------------------------------------------------------

    if closest_component:

        bbox_min = np.array(
            closest_component["bbox"]["min"],
            dtype=np.float64
        )

        bbox_max = np.array(
            closest_component["bbox"]["max"],
            dtype=np.float64
        )

        nominal_center_y = float(
            closest_component["nominal_center"][1]
        )

        # Expand nominal component region so a shifted
        # component is still captured.
        X_MARGIN = 2.0
        Y_MARGIN = 50.0
        Z_MARGIN = 2.0

        # Ignore points very close to the base plate.
        # The stiffener starts at approximately Z = 12 mm.
        MIN_COMPONENT_Z = bbox_min[2] + 5.0

        component_mask = (
            (scan_points[:, 0] >= bbox_min[0] - X_MARGIN) &
            (scan_points[:, 0] <= bbox_max[0] + X_MARGIN) &

            (scan_points[:, 1] >= bbox_min[1] - Y_MARGIN) &
            (scan_points[:, 1] <= bbox_max[1] + Y_MARGIN) &

            (scan_points[:, 2] >= MIN_COMPONENT_Z) &
            (scan_points[:, 2] <= bbox_max[2] + Z_MARGIN)
        )

        component_points = scan_points[component_mask]
        result["points_used"] = len(component_points)
        result["nominal_center_y_mm"] = nominal_center_y

        print("\n========================================")
        print("SHIPQA - COMPONENT SHIFT ESTIMATION")
        print("========================================")

        print(
            f"Component           : "
            f"{closest_component['assembly_id']}"
        )

        print(
            f"Points used         : "
            f"{len(component_points):,}"
        )

        if len(component_points) > 0:

            y_values = component_points[:, 1]

            # Robust estimates of the two transverse edges.
            # Avoid using absolute min/max because scanner noise
            # can produce outliers.
            y_low = np.percentile(y_values, 2)
            y_high = np.percentile(y_values, 98)

            observed_center_y = (
                y_low + y_high
            ) / 2.0

            estimated_shift_y = (
                observed_center_y - nominal_center_y
            )

            print(
                f"Nominal center Y    : "
                f"{nominal_center_y:.3f} mm"
            )

            print(
                f"Observed Y envelope : "
                f"{y_low:.3f} -> {y_high:.3f} mm"
            )

            print(
                f"Observed center Y   : "
                f"{observed_center_y:.3f} mm"
            )

            print(
                f"Estimated Y shift   : "
                f"{estimated_shift_y:+.3f} mm"
            )

        print("========================================\n")

        if len(component_points) > 0:
            result.update(status="estimated", observed_center_y_mm=float(observed_center_y),
                          estimated_shift_y_mm=float(estimated_shift_y),
                          observed_y_envelope_mm=[float(y_low), float(y_high)])
            try:
                pose = estimate_translation_xyz(
                    scan_points, closest_component, mesh_path,
                    float(estimated_shift_y), component_points)
                if pose is not None:
                    result["estimated_translation_xyz_mm"] = pose[
                        "estimated_translation_xyz_mm"]
                    pose_status = ("estimated" if all(
                        value is not None
                        for value in pose["estimated_translation_xyz_mm"])
                        else "partial_estimate")
                    result["component_pose"] = dict(pose, status=pose_status)
            except ValueError as error:
                # Historical OBJ fixtures have no group records. Their validated
                # Y result remains available; XYZ pose is explicitly unavailable.
                result["component_pose"] = {
                    "status": "unsupported_reference",
                    "reason": str(error),
                }

    return result


def run_prediction(scan_path, mesh_path, manifest_path, output_dir, registration=False,
                   initial_alignment_path=None, filter_isolated_outliers=False):
    """Write one prediction into a new directory. Never replace an existing result."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    with (output_dir / "analysis.log").open("x", encoding="utf-8") as log:
        with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            try:
                result = analyze_scan(
                    scan_path, mesh_path, manifest_path,
                    output_dir / "defect_candidates.xyz", registration=registration,
                    registered_scan_output=(
                        output_dir / "registered_scan.xyz" if registration else None),
                    initial_alignment_path=initial_alignment_path,
                    filter_isolated_outliers=filter_isolated_outliers,
                )
            except Exception as error:
                traceback.print_exc()
                result = {"schema_version": 1, "status": "error", "units": "mm",
                          "error": str(error), "candidate_count": None,
                          "detected_component": None, "estimated_shift_y_mm": None,
                          "estimated_translation_xyz_mm": None,
                          "points_used": None}
    with (output_dir / "prediction.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main():
    parser = argparse.ArgumentParser(description="ShipQA geometry-only prediction (millimetres)")
    parser.add_argument("--scan", type=Path, default=SCAN_PATH)
    parser.add_argument("--mesh", type=Path, default=MESH_PATH)
    parser.add_argument("--manifest", type=Path, default=MANIFEST_PATH)
    parser.add_argument("--output-dir", type=Path,
                        help="New directory for prediction.json, candidates and analysis.log")
    parser.add_argument("--register", action="store_true",
                        help="Estimate a global rigid scan-to-CAD transform before QA")
    parser.add_argument("--initial-alignment", type=Path,
                        help="Survey/scanner/manual coarse-alignment JSON")
    parser.add_argument("--filter-isolated-outliers", action="store_true",
                        help="Remove statistically isolated scan points before registration and QA")
    args = parser.parse_args()
    if args.output_dir is None:
        analyze_scan(
            args.scan, args.mesh, args.manifest, registration=args.register,
            initial_alignment_path=args.initial_alignment,
            filter_isolated_outliers=args.filter_isolated_outliers)
        return 0
    result = run_prediction(
        args.scan, args.mesh, args.manifest, args.output_dir, registration=args.register,
        initial_alignment_path=args.initial_alignment,
        filter_isolated_outliers=args.filter_isolated_outliers)
    print("Prediction status: {}".format(result["status"]))
    print("Output: {}".format(args.output_dir.resolve()))
    if result["status"] == "error":
        return 1
    if result["status"] == "registration_rejected":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
