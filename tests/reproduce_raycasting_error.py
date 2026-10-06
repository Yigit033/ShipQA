r"""Minimal standalone reproduction: one point, one triangle, no OBJ or ShipQA.

Run with .venv\Scripts\python.exe -B tests/reproduce_raycasting_error.py
On the installed Open3D 0.20.0 CPU build: 3.414161 mm instead of 0.431500 mm.
"""

import numpy as np
import open3d as o3d


def main():
    vertices = np.array([[4000, 1240, 172], [0, 1240, 162], [0, 1240, 172]], dtype=np.float32)
    point = np.array([[1364.6434, 1240.4315, 168.5501]], dtype=np.float64)
    projected = np.array([point[0, 0], 1240, point[0, 2]])
    # Independent plane projection with an interior check, all in float64.
    a, b, c = vertices.astype(np.float64)
    u, v = np.linalg.lstsq(np.column_stack([b - a, c - a]), projected - a, rcond=None)[0]
    assert u > 0 and v > 0 and u + v < 1
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(vertices), o3d.core.Tensor([[0, 1, 2]], dtype=o3d.core.Dtype.UInt32))
    query = o3d.core.Tensor(point.astype(np.float32))
    print("Open3D:", o3d.__version__)
    print("RaycastingScene distance [mm]:", scene.compute_distance(query).numpy()[0])
    print("Analytic distance [mm]:", np.linalg.norm(point[0] - projected))
    print("Input float32 rounding [mm]:", np.linalg.norm(point - point.astype(np.float32)))
    print("Closest point returned:", scene.compute_closest_points(query)["points"].numpy()[0])


if __name__ == "__main__":
    main()
