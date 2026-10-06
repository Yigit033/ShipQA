"""Rigid scan-to-CAD registration for the external Python engine.

Coordinates and distances are millimetres.  The returned homogeneous transform
maps raw scan coordinates into the nominal CAD coordinate system.  This first
controlled implementation assumes that scan and CAD axes are already roughly
parallel; centroid translation provides the coarse initialization and robust,
multi-stage point-to-plane ICP provides the fine alignment.
"""

import math

import numpy as np
import open3d as o3d


TARGET_SAMPLE_COUNT = 100000
TARGET_SAMPLE_SEED = 42
ICP_STAGES = (
    {"max_correspondence_mm": 150.0, "tukey_scale_mm": 10.0, "max_iterations": 100},
    {"max_correspondence_mm": 50.0, "tukey_scale_mm": 5.0, "max_iterations": 100},
    {"max_correspondence_mm": 15.0, "tukey_scale_mm": 2.0, "max_iterations": 100},
)
OUTLIER_RADIUS_MM = 50.0
OUTLIER_MIN_POINTS = 4


def apply_transform(points, transform):
    """Apply a column-vector homogeneous transform to an (N, 3) point array."""
    points = np.asarray(points, dtype=np.float64)
    transform = np.asarray(transform, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Expected points with shape (N, 3)")
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Expected a finite 4x4 transform")
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotation_angle_degrees(rotation):
    """Return the principal rotation angle of a 3x3 rotation matrix."""
    value = (float(np.trace(rotation)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(value, -1.0, 1.0))))


def validate_rigid_transform(transform):
    """Validate and return a proper finite homogeneous rigid transform."""
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Expected a finite 4x4 transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise ValueError("Invalid homogeneous transform bottom row")
    rotation = transform[:3, :3]
    determinant = float(np.linalg.det(rotation))
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    if abs(determinant - 1.0) > 1e-6 or orthogonality_error > 1e-6:
        raise ValueError("Transform must contain a proper rigid rotation")
    return transform


def rigid_transform_from_correspondences(scan_points, cad_points):
    """Estimate scan-to-CAD transform from at least three non-collinear pairs."""
    scan_points = np.asarray(scan_points, dtype=np.float64)
    cad_points = np.asarray(cad_points, dtype=np.float64)
    if (scan_points.ndim != 2 or scan_points.shape[1] != 3
            or cad_points.shape != scan_points.shape or len(scan_points) < 3):
        raise ValueError("Correspondences require matching (N, 3) arrays with N >= 3")
    if not np.isfinite(scan_points).all() or not np.isfinite(cad_points).all():
        raise ValueError("Correspondence coordinates must be finite")
    scan_center = scan_points.mean(axis=0)
    cad_center = cad_points.mean(axis=0)
    scan_centered = scan_points - scan_center
    cad_centered = cad_points - cad_center
    if np.linalg.matrix_rank(scan_centered, tol=1e-9) < 2:
        raise ValueError("Correspondence points must not be collinear")
    u, _, vt = np.linalg.svd(scan_centered.T @ cad_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = cad_center - rotation @ scan_center
    return validate_rigid_transform(transform)


def initial_transform_from_spec(spec):
    """Build a coarse scan-to-CAD transform from a field-input JSON object."""
    if not isinstance(spec, dict) or spec.get("units") != "mm":
        raise ValueError("Initial alignment must be a JSON object in millimetres")
    method = spec.get("method")
    if method == "transform_matrix":
        transform = validate_rigid_transform(spec.get("transform_scan_to_cad"))
        details = {"method": method, "source": spec.get("source", "unspecified")}
    elif method == "corresponding_points":
        scan_points = np.asarray(spec.get("scan_points_mm"), dtype=np.float64)
        cad_points = np.asarray(spec.get("cad_points_mm"), dtype=np.float64)
        transform = rigid_transform_from_correspondences(scan_points, cad_points)
        residual = apply_transform(scan_points, transform) - cad_points
        details = {
            "method": method,
            "source": spec.get("source", "unspecified"),
            "correspondence_count": len(scan_points),
            "correspondence_rmse_mm": float(
                np.sqrt(np.mean(np.einsum("ij,ij->i", residual, residual)))),
            "correspondence_max_error_mm": float(np.linalg.norm(residual, axis=1).max()),
        }
    else:
        raise ValueError("Initial alignment method must be transform_matrix or corresponding_points")
    return transform, details


def remove_isolated_outliers(points):
    """Remove points without local radius support; return points and indices."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) <= OUTLIER_MIN_POINTS:
        raise ValueError("Too few points for isolated-outlier filtering")
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    _, retained = cloud.remove_radius_outlier(
        nb_points=OUTLIER_MIN_POINTS, radius=OUTLIER_RADIUS_MM)
    retained = np.asarray(retained, dtype=np.int64)
    if len(retained) < 3:
        raise RuntimeError("Outlier filtering retained fewer than three points")
    return points[retained], retained


def _validate_geometry(scan_points, vertices, triangles):
    scan_points = np.asarray(scan_points, dtype=np.float64)
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles)
    if scan_points.ndim != 2 or scan_points.shape[1] != 3 or len(scan_points) < 3:
        raise ValueError("Registration requires at least three XYZ scan points")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3:
        raise ValueError("Registration requires nominal mesh vertices")
    if triangles.ndim != 2 or triangles.shape[1] != 3 or len(triangles) == 0:
        raise ValueError("Registration requires nominal mesh triangles")
    if not np.issubdtype(triangles.dtype, np.integer):
        raise ValueError("Triangle indices must be integers")
    if triangles.min() < 0 or triangles.max() >= len(vertices):
        raise ValueError("Triangle index is out of bounds")
    if not np.isfinite(scan_points).all() or not np.isfinite(vertices).all():
        raise ValueError("Registration geometry must be finite")
    return scan_points, vertices, triangles.astype(np.int32, copy=False)


def _sample_nominal_surface(vertices, triangles):
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(triangles))
    mesh.compute_vertex_normals()
    o3d.utility.random.seed(TARGET_SAMPLE_SEED)
    target = mesh.sample_points_uniformly(
        number_of_points=TARGET_SAMPLE_COUNT, use_triangle_normal=True)
    if not target.has_normals() or len(target.points) != TARGET_SAMPLE_COUNT:
        raise RuntimeError("Could not sample the nominal surface with normals")
    return target


def register_scan_to_nominal(scan_points, vertices, triangles,
                             initial_transform=None, filter_isolated_outliers=False):
    """Return scan points in CAD coordinates plus traceable ICP diagnostics.

    No component IDs, defect thresholds, expected shifts or ground truth enter
    this calculation.  The algorithm estimates one global rigid transform.
    """
    scan_points, vertices, triangles = _validate_geometry(
        scan_points, vertices, triangles)
    input_point_count = len(scan_points)
    retained_indices = np.arange(input_point_count, dtype=np.int64)
    if filter_isolated_outliers:
        scan_points, retained_indices = remove_isolated_outliers(scan_points)

    target = _sample_nominal_surface(vertices, triangles)
    target_points = np.asarray(target.points)
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scan_points))

    if initial_transform is None:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, 3] = target_points.mean(axis=0) - scan_points.mean(axis=0)
        initialization_method = "centroid_translation_with_axis_prior"
    else:
        transform = validate_rigid_transform(initial_transform).copy()
        initialization_method = "external_coarse_alignment"
    coarse_transform = transform.copy()
    stages = []

    for settings in ICP_STAGES:
        loss = o3d.pipelines.registration.TukeyLoss(
            k=settings["tukey_scale_mm"])
        estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-8,
            relative_rmse=1e-8,
            max_iteration=settings["max_iterations"],
        )
        result = o3d.pipelines.registration.registration_icp(
            source,
            target,
            settings["max_correspondence_mm"],
            transform,
            estimator,
            criteria,
        )
        transform = np.asarray(result.transformation, dtype=np.float64)
        stages.append({
            **settings,
            "fitness": float(result.fitness),
            "inlier_rmse_mm": float(result.inlier_rmse),
        })

    transform = validate_rigid_transform(transform)
    rotation = transform[:3, :3]
    determinant = float(np.linalg.det(rotation))
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    registered_points = apply_transform(scan_points, transform)
    diagnostics = {
        "status": "completed",
        "method": "coarse_initialization_then_robust_point_to_plane_icp",
        "initialization_method": initialization_method,
        "coordinate_convention": "scan_to_nominal_cad_column_vector",
        "units": "mm",
        "axis_prior": "scan_and_cad_axes_must_be_roughly_parallel",
        "target_sample_count": TARGET_SAMPLE_COUNT,
        "target_sample_seed": TARGET_SAMPLE_SEED,
        "input_point_count": input_point_count,
        "retained_point_count": len(scan_points),
        "removed_isolated_point_count": input_point_count - len(scan_points),
        "retained_point_fraction": float(len(scan_points) / input_point_count),
        "outlier_filter": {
            "enabled": bool(filter_isolated_outliers),
            "method": "radius_neighbor_support",
            "radius_mm": OUTLIER_RADIUS_MM,
            "minimum_points": OUTLIER_MIN_POINTS,
        },
        "coarse_transform_scan_to_cad": coarse_transform.tolist(),
        "transform_scan_to_cad": transform.tolist(),
        "estimated_translation_mm": transform[:3, 3].tolist(),
        "estimated_rotation_angle_deg": rotation_angle_degrees(rotation),
        "rotation_determinant": determinant,
        "rotation_orthogonality_error": orthogonality_error,
        "stages": stages,
    }
    return registered_points, retained_indices, diagnostics
