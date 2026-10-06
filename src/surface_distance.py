"""Unsigned point-to-triangle surface distances in millimetres, using float64.

Exhaustive O(points * triangles) evaluation is intentional for the small prototype.
No candidate triangle is selected using the inaccurate float32 closest-point query.
Working storage is O(points), not a points-by-triangles array.
"""

import numpy as np


def compute_surface_deviation(points, vertices, triangles):
    """Return minimum distance to the closed triangular surfaces, not solid volumes.

For each triangle the minimum lies on an edge (including its endpoints), or at
the perpendicular face projection if that projection is inside all edge half-spaces.
The half-space tests use cross products; they avoid subtracting nearly equal
products of dot products to calculate barycentric coordinates on thin triangles.
Collapsed triangles are treated as their segments/points. No physical tolerance,
unit conversion, threshold, component label or known displacement enters this code.
"""
    points = np.asarray(points, dtype=np.float64)
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Expected points with shape (N, 3)")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0:
        raise ValueError("Expected nonempty vertices with shape (V, 3)")
    if triangles.ndim != 2 or triangles.shape[1] != 3 or len(triangles) == 0:
        raise ValueError("Expected nonempty triangles with shape (T, 3)")
    if not np.issubdtype(triangles.dtype, np.integer):
        raise ValueError("Triangle indices must be integers")
    if triangles.min() < 0 or triangles.max() >= len(vertices):
        raise ValueError("Triangle index is out of bounds")
    if not np.isfinite(points).all() or not np.isfinite(vertices).all():
        raise ValueError("Points and vertices must be finite")

    best_squared = np.full(len(points), np.inf, dtype=np.float64)
    # Fail visibly for coordinates outside the representable arithmetic range.
    with np.errstate(over="raise", invalid="raise", divide="raise"):
        for face in triangles:
            a, b, c = vertices[face]
            normal = np.cross(b - a, c - a)
            normal_length = np.linalg.norm(normal)
            if normal_length > 0:
                normal /= normal_length
            inside = np.ones(len(points), dtype=bool)
            for start, end in ((a, b), (b, c), (c, a)):
                edge = end - start
                relative = points - start
                edge_squared = np.dot(edge, edge)
                if edge_squared > 0:
                    t = np.clip(np.einsum("ij,j->i", relative, edge) / edge_squared, 0.0, 1.0)
                    residual = relative - t[:, None] * edge
                else:
                    residual = relative
                squared = np.einsum("ij,ij->i", residual, residual)
                np.minimum(best_squared, squared, out=best_squared)
                if normal_length > 0:
                    # The point's normal component does not affect this triple product.
                    side = np.einsum("ij,j->i", np.cross(edge, relative), normal)
                    inside &= side >= 0.0
            if normal_length > 0 and inside.any():
                height = np.einsum("ij,j->i", points[inside] - a, normal)
                best_squared[inside] = np.minimum(best_squared[inside], height * height)
    distances = np.sqrt(best_squared)
    if not np.isfinite(distances).all():
        raise ValueError("Non-finite surface distance")
    return distances
