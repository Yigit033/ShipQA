"""Local rigid rotation estimation for an identified engineering component."""

import numpy as np
import open3d as o3d

from component_pose import (load_grouped_obj_component, sample_triangle_mesh,
                            select_component_region)


LOCAL_POSE_MARGIN_MM = 80.0
LOCAL_NOMINAL_SAMPLE_COUNT = 50000
MIN_LOCAL_ROTATION_FITNESS = 0.95
MAX_LOCAL_ROTATION_RMSE_MM = 5.0


def rotation_matrix_to_euler_xyz_degrees(rotation):
    """Decompose Rz*Ry*Rx into signed XYZ Euler angles in degrees."""
    rotation = np.asarray(rotation, dtype=np.float64)
    sy = -float(rotation[2, 0])
    sy = float(np.clip(sy, -1.0, 1.0))
    ry = np.arcsin(sy)
    cy = np.cos(ry)
    if abs(cy) > 1e-10:
        rx = np.arctan2(rotation[2, 1], rotation[2, 2])
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
        rz = 0.0
    return np.degrees([rx, ry, rz]).tolist()


def estimate_component_rotation(scan_points, component, mesh_path):
    """Estimate nominal-to-observed local rigid pose without experiment metadata."""
    vertices, triangles = load_grouped_obj_component(mesh_path, component["part_ids"])
    observed = select_component_region(
        scan_points, component, margin_mm=LOCAL_POSE_MARGIN_MM)
    if len(observed) < 10:
        return {"status": "insufficient_component_points",
                "observed_points_used": int(len(observed))}
    nominal, normals = sample_triangle_mesh(
        vertices, triangles, count=LOCAL_NOMINAL_SAMPLE_COUNT,
        return_normals=True)
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(observed))
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(nominal))
    target.normals = o3d.utility.Vector3dVector(normals)
    transform = np.eye(4)
    stages = []
    for distance, robust_scale in ((80.0, 30.0), (30.0, 10.0), (10.0, 3.0)):
        estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane(
            o3d.pipelines.registration.TukeyLoss(k=robust_scale))
        result = o3d.pipelines.registration.registration_icp(
            source, target, distance, transform, estimator,
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-8, relative_rmse=1e-8, max_iteration=80))
        transform = np.asarray(result.transformation, dtype=np.float64)
        stages.append({"max_correspondence_mm": distance,
                       "robust_loss_scale_mm": robust_scale,
                       "fitness": float(result.fitness),
                       "inlier_rmse_mm": float(result.inlier_rmse)})
    nominal_to_observed = np.linalg.inv(transform)
    rotation = nominal_to_observed[:3, :3]
    center = np.asarray(component["nominal_center"], dtype=np.float64)
    observed_center = rotation @ center + nominal_to_observed[:3, 3]
    final_stage = stages[-1]
    quality_checks = {
        "fitness_at_least_0_95": final_stage["fitness"] >= MIN_LOCAL_ROTATION_FITNESS,
        "rmse_at_most_5_mm": final_stage["inlier_rmse_mm"] <= MAX_LOCAL_ROTATION_RMSE_MM,
    }
    quality_status = "pass_controlled" if all(quality_checks.values()) else "fail"
    return {
        "status": "estimated" if quality_status == "pass_controlled"
        else "rejected_low_quality",
        "method": "robust_component_point_to_plane_icp",
        "transform_nominal_to_observed": nominal_to_observed.tolist(),
        "rotation_matrix_nominal_to_observed": rotation.tolist(),
        "estimated_rotation_xyz_deg": rotation_matrix_to_euler_xyz_degrees(rotation),
        "estimated_center_translation_mm": (observed_center - center).tolist(),
        "observed_points_used": int(len(observed)),
        "nominal_points_used": int(len(nominal)),
        "stages": stages,
        "quality_status": quality_status,
        "quality_checks": quality_checks,
        "quality_thresholds": {
            "minimum_fitness": MIN_LOCAL_ROTATION_FITNESS,
            "maximum_rmse_mm": MAX_LOCAL_ROTATION_RMSE_MM,
        },
        "scope": "Controlled synthetic qualification; field thresholds require real-data validation",
    }
