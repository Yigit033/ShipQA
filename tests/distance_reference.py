"""Independent test oracle: exhaustive Decimal triangle distances, no Open3D.

The reference enumerates vertices, clamped edges and the planar stationary point.
Its barycentric solve uses 60-digit arithmetic rather than the production kernel's
float64 edge half-spaces. It intentionally prioritizes verification over speed.
"""

from decimal import Decimal, localcontext
from pathlib import Path

import numpy as np


def read_obj_triangles(path):
    vertices, faces = [], []
    for line in Path(path).read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append([float(x) for x in fields[1:4]])
        elif fields[0] == "f":
            if len(fields) != 4:
                raise ValueError("This audit requires exported triangular OBJ faces")
            face = [int(s.split("/")[0]) for s in fields[1:]]
            if any(i <= 0 or i > len(vertices) for i in face):
                raise ValueError("Invalid 1-based OBJ indexing")
            faces.append([i - 1 for i in face])
    return np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def decimal_triangle_distance(point, triangle):
    def vector(values):
        return [Decimal(str(float(x))) for x in values]
    def sub(a, b):
        return [x - y for x, y in zip(a, b)]
    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))
    def along(a, direction, t):
        return [x + t * y for x, y in zip(a, direction)]
    with localcontext() as context:
        context.prec = 60
        p = vector(point)
        a, b, c = [vector(v) for v in triangle]
        candidates = [a, b, c]
        for start, end in ((a, b), (b, c), (c, a)):
            edge = sub(end, start)
            squared_length = dot(edge, edge)
            if squared_length:
                t = min(Decimal(1), max(Decimal(0), dot(sub(p, start), edge) / squared_length))
                candidates.append(along(start, edge, t))
        ab, ac, ap = sub(b, a), sub(c, a), sub(p, a)
        aa, bb, cc = dot(ab, ab), dot(ab, ac), dot(ac, ac)
        rhs_a, rhs_b = dot(ap, ab), dot(ap, ac)
        determinant = aa * cc - bb * bb
        if determinant:
            u = (rhs_a * cc - rhs_b * bb) / determinant
            v = (rhs_b * aa - rhs_a * bb) / determinant
            if u >= 0 and v >= 0 and u + v <= 1:
                candidates.append(along(along(a, ab, u), ac, v))
        squared_distances = [dot(sub(p, q), sub(p, q)) for q in candidates]
        index = min(range(len(candidates)), key=squared_distances.__getitem__)
        return float(squared_distances[index].sqrt()), np.array([float(x) for x in candidates[index]])


def brute_force_obj_distance(point, vertices, faces):
    best = (float("inf"), None, None)
    for index, face in enumerate(faces):
        distance, closest = decimal_triangle_distance(point, vertices[face])
        if distance < best[0]:
            best = (distance, closest, index)
    return best


def box_surface_distances(points, boxes):
    """Secondary oracle for this panel only; unsigned distance to all box surfaces."""
    result = np.full(len(points), np.inf)
    for lo, hi in boxes:
        lo, hi = np.asarray(lo), np.asarray(hi)
        q = np.abs(points - (lo + hi) / 2) - (hi - lo) / 2
        signed = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
        result = np.minimum(result, np.abs(signed))
    return result
