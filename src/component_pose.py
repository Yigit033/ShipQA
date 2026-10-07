"""Component-specific translation estimation from measured and nominal geometry.

The OBJ group names carry PART_ID values. Ground truth, expected axes and case
labels are deliberately absent from this module.
"""

from pathlib import Path

import numpy as np


POSE_MARGIN_MM = 50.0
LOW_PERCENTILE = 2.0
HIGH_PERCENTILE = 98.0
NOMINAL_SAMPLE_COUNT = 100000
NOMINAL_SAMPLE_SEED = 1729
NORMAL_ALIGNMENT_MIN = 0.95
MIN_EXPECTED_AXIS_SUPPORT_POINTS = 10.0


def load_grouped_obj_component(path, part_ids):
    """Return component vertices/triangles selected by OBJ ``g PART_ID`` records."""
    vertices = []
    faces = []
    active_group = None
    wanted = set(part_ids)
    with Path(path).open(encoding="utf-8") as stream:
        for raw in stream:
            fields = raw.strip().split()
            if not fields:
                continue
            if fields[0] == "v" and len(fields) >= 4:
                vertices.append([float(value) for value in fields[1:4]])
            elif fields[0] == "g":
                active_group = fields[1] if len(fields) == 2 else None
            elif fields[0] == "f" and active_group in wanted:
                if len(fields) != 4:
                    raise ValueError("Component pose requires triangular OBJ faces")
                face = []
                for token in fields[1:]:
                    index = int(token.split("/")[0])
                    if index <= 0:
                        raise ValueError("Component pose requires positive OBJ indices")
                    face.append(index - 1)
                faces.append(face)
    if not faces:
        raise ValueError("Nominal OBJ has no grouped faces for component parts: {}".format(
            ", ".join(sorted(wanted))))
    all_vertices = np.asarray(vertices, dtype=np.float64)
    global_faces = np.asarray(faces, dtype=np.int64)
    used, inverse = np.unique(global_faces.reshape(-1), return_inverse=True)
    return all_vertices[used], inverse.reshape((-1, 3))


def sample_triangle_mesh(vertices, triangles, count=NOMINAL_SAMPLE_COUNT,
                         seed=NOMINAL_SAMPLE_SEED, return_normals=False):
    """Deterministically sample triangle area for a nominal quantile reference."""
    tri = vertices[triangles]
    areas = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0],
                                    tri[:, 2] - tri[:, 0]), axis=1) * 0.5
    total = float(areas.sum())
    if not np.isfinite(total) or total <= 0.0:
        raise ValueError("Component nominal mesh has no positive surface area")
    rng = np.random.default_rng(seed)
    chosen = rng.choice(len(tri), size=count, p=areas / total)
    selected = tri[chosen]
    first = rng.random(count)
    second = rng.random(count)
    root = np.sqrt(first)
    points = ((1.0 - root)[:, None] * selected[:, 0]
              + (root * (1.0 - second))[:, None] * selected[:, 1]
              + (root * second)[:, None] * selected[:, 2])
    if not return_normals:
        return points
    cross = np.cross(selected[:, 1] - selected[:, 0],
                     selected[:, 2] - selected[:, 0])
    length = np.linalg.norm(cross, axis=1)
    normals = np.divide(cross, length[:, None], out=np.zeros_like(cross),
                        where=length[:, None] > 0.0)
    return points, normals


def select_component_region(scan_points, component, margin_mm=POSE_MARGIN_MM):
    """Select a broad component region while excluding the nominal plate skin."""
    bbox_min = np.asarray(component["bbox"]["min"], dtype=np.float64)
    bbox_max = np.asarray(component["bbox"]["max"], dtype=np.float64)
    minimum_component_z = bbox_min[2] + 5.0
    mask = (
        (scan_points[:, 0] >= bbox_min[0] - margin_mm)
        & (scan_points[:, 0] <= bbox_max[0] + margin_mm)
        & (scan_points[:, 1] >= bbox_min[1] - margin_mm)
        & (scan_points[:, 1] <= bbox_max[1] + margin_mm)
        & (scan_points[:, 2] >= minimum_component_z)
        & (scan_points[:, 2] <= bbox_max[2] + margin_mm)
    )
    return scan_points[mask]


def component_translation_axis_support(vertices, triangles, observed_point_count):
    """Report which translation axes have enough normal-facing surface evidence."""
    tri = vertices[triangles]
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    double_area = np.linalg.norm(cross, axis=1)
    normals = np.divide(
        cross, double_area[:, None], out=np.zeros_like(cross),
        where=double_area[:, None] > 0.0)
    total_area = float((double_area * 0.5).sum())
    if not np.isfinite(total_area) or total_area <= 0.0:
        raise ValueError("Component nominal mesh has no positive surface area")
    axis_status = {}
    for axis, name in enumerate("xyz"):
        supported_area = float((double_area[
            np.abs(normals[:, axis]) >= NORMAL_ALIGNMENT_MIN] * 0.5).sum())
        area_fraction = supported_area / total_area
        expected_points = float(observed_point_count * area_fraction)
        observable = expected_points >= MIN_EXPECTED_AXIS_SUPPORT_POINTS
        axis_status[name] = {
            "status": "estimated" if observable else "insufficient_geometric_support",
            "normal_aligned_surface_fraction": area_fraction,
            "expected_support_points": expected_points,
            "minimum_expected_support_points": MIN_EXPECTED_AXIS_SUPPORT_POINTS,
            "normal_alignment_minimum": NORMAL_ALIGNMENT_MIN,
        }
    return axis_status


def estimate_translation_xyz(scan_points, component, mesh_path,
                             legacy_y_shift_mm, legacy_component_points):
    """Estimate XYZ translation without knowing which axis was displaced.

    The validated Y value is retained exactly. X and Z compare robust observed
    envelopes against deterministic samples of the component's grouped nominal
    faces. Points close to the plate are excluded as in the legacy estimator.
    """
    bbox_min = np.asarray(component["bbox"]["min"], dtype=np.float64)
    bbox_max = np.asarray(component["bbox"]["max"], dtype=np.float64)
    vertices, triangles = load_grouped_obj_component(mesh_path, component["part_ids"])
    observed = select_component_region(scan_points, component)
    if len(observed) == 0:
        return None
    nominal = sample_triangle_mesh(vertices, triangles)
    observed_envelope = np.percentile(
        observed, [LOW_PERCENTILE, HIGH_PERCENTILE], axis=0)
    nominal_envelope = np.percentile(
        nominal, [LOW_PERCENTILE, HIGH_PERCENTILE], axis=0)
    observed_center = observed_envelope.mean(axis=0)
    nominal_center = nominal_envelope.mean(axis=0)
    envelope_translation = observed_center - nominal_center

    axis_status = component_translation_axis_support(
        vertices, triangles, len(observed))
    translation = [None, None, None]
    for axis, name in enumerate("xyz"):
        if axis_status[name]["status"] == "estimated":
            translation[axis] = float(envelope_translation[axis])

    # Preserve the validated Y estimator byte-for-byte when Y is observable.
    if translation[1] is not None:
        translation[1] = float(legacy_y_shift_mm)
        axis_status["y"]["method"] = "validated_legacy_percentile_envelope"

    # The lower Z region may be hidden by or intersect the plate. The upper
    # component surface remains observable for both signs of rigid Z translation.
    if translation[2] is not None:
        translation[2] = float(observed_envelope[1, 2] - nominal_envelope[1, 2])
        axis_status["z"]["method"] = "upper_component_surface_percentile"

    if translation[0] is not None:
        axis_status["x"]["method"] = "two_sided_percentile_envelope"
    return {
        "estimated_translation_xyz_mm": translation,
        "axis_status": axis_status,
        "pose_points_used": int(len(observed)),
        "legacy_y_points_used": int(len(legacy_component_points)),
        "observed_pose_envelope_mm": observed_envelope.tolist(),
        "nominal_pose_envelope_mm": nominal_envelope.tolist(),
        "method": "grouped_component_surface_percentile_envelope",
        "percentiles": [LOW_PERCENTILE, HIGH_PERCENTILE],
        "limitations": ["Negative Z translation can intersect the nominal base plate"],
    }
