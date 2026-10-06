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
                         seed=NOMINAL_SAMPLE_SEED):
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
    return ((1.0 - root)[:, None] * selected[:, 0]
            + (root * (1.0 - second))[:, None] * selected[:, 1]
            + (root * second)[:, None] * selected[:, 2])


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
    minimum_component_z = bbox_min[2] + 5.0
    mask = (
        (scan_points[:, 0] >= bbox_min[0] - POSE_MARGIN_MM)
        & (scan_points[:, 0] <= bbox_max[0] + POSE_MARGIN_MM)
        & (scan_points[:, 1] >= bbox_min[1] - POSE_MARGIN_MM)
        & (scan_points[:, 1] <= bbox_max[1] + POSE_MARGIN_MM)
        & (scan_points[:, 2] >= minimum_component_z)
        & (scan_points[:, 2] <= bbox_max[2] + POSE_MARGIN_MM)
    )
    observed = scan_points[mask]
    if len(observed) == 0:
        return None
    nominal = sample_triangle_mesh(vertices, triangles)
    observed_envelope = np.percentile(
        observed, [LOW_PERCENTILE, HIGH_PERCENTILE], axis=0)
    nominal_envelope = np.percentile(
        nominal, [LOW_PERCENTILE, HIGH_PERCENTILE], axis=0)
    observed_center = observed_envelope.mean(axis=0)
    nominal_center = nominal_envelope.mean(axis=0)
    translation = observed_center - nominal_center
    # Preserve the validated Y estimator byte-for-byte in the public vector.
    translation[1] = legacy_y_shift_mm
    return {
        "estimated_translation_xyz_mm": translation.tolist(),
        "pose_points_used": int(len(observed)),
        "legacy_y_points_used": int(len(legacy_component_points)),
        "observed_pose_envelope_mm": observed_envelope.tolist(),
        "nominal_pose_envelope_mm": nominal_envelope.tolist(),
        "method": "grouped_component_surface_percentile_envelope",
        "percentiles": [LOW_PERCENTILE, HIGH_PERCENTILE],
        "limitations": [
            "X translation of a long prismatic stiffener is weakly observable at its ends",
            "Negative Z translation can intersect the nominal base plate",
        ],
    }
