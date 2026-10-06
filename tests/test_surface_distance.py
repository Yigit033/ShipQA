import itertools
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from surface_distance import compute_surface_deviation
from distance_reference import decimal_triangle_distance, brute_force_obj_distance, read_obj_triangles


class SurfaceDistanceTests(unittest.TestCase):
    def test_face_edge_vertex_and_surface_cases(self):
        triangle = np.array([[0,0,0], [10,0,0], [0,10,0]], dtype=float)
        points = np.array([[2,2,3], [6,6,0], [-3,-4,0], [2,3,0], [5,5,0], [11,0,2]], dtype=float)
        expected = [3, np.sqrt(2), 5, 0, 0, np.sqrt(5)]
        actual = compute_surface_deviation(points, triangle, [[0,1,2]])
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)
        for p, value in zip(points, expected):
            self.assertAlmostEqual(decimal_triangle_distance(p, triangle)[0], value, places=12)

    def test_thin_triangle_all_vertex_orders(self):
        triangle = np.array([[4000,1240,172], [0,1240,162], [0,1240,172]], dtype=float)
        point = np.array([[1364.6434,1240.4315,168.5501]])
        for permutation in itertools.permutations(range(3)):
            actual = compute_surface_deviation(point, triangle, [permutation])[0]
            oracle = decimal_triangle_distance(point[0], triangle[list(permutation)])[0]
            self.assertAlmostEqual(actual, 0.4315, places=10)
            self.assertAlmostEqual(actual, oracle, places=10)

    def test_rigid_transform_and_scale_invariance(self):
        triangle = np.array([[4000,1240,172], [0,1240,162], [0,1240,172]], dtype=float)
        points = np.array([[1364.6434,1240.4315,168.5501], [-3,1240,162], [3999,1240,0]])
        base = compute_surface_deviation(points, triangle, [[0,1,2]])
        rotation, _ = np.linalg.qr(np.random.default_rng(19).normal(size=(3,3)))
        translation = np.array([1e6, -2e6, 3e6])
        actual = compute_surface_deviation(points @ rotation + translation, triangle @ rotation + translation, [[0,1,2]])
        np.testing.assert_allclose(actual, base, atol=1e-8, rtol=0)
        for scale in [0.001, 1000.0]:
            actual = compute_surface_deviation(points * scale, triangle * scale, [[0,1,2]]) / scale
            np.testing.assert_allclose(actual, base, atol=1e-9, rtol=0)

    def test_degenerate_triangles_are_segments_or_points(self):
        for triangle in [np.array([[0,0,0],[10,0,0],[5,0,0]], dtype=float),
                         np.array([[1,2,3],[1,2,3],[1,2,3]], dtype=float),
                         np.array([[0,0,0],[0,0,0],[10,0,0]], dtype=float)]:
            points = np.array([[2,3,4], [-1,2,0], [12,0,0]], dtype=float)
            actual = compute_surface_deviation(points, triangle, [[0,1,2]])
            expected = [decimal_triangle_distance(p,triangle)[0] for p in points]
            np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)

    def test_random_mesh_against_decimal_exhaustive_oracle(self):
        rng = np.random.default_rng(27)
        vertices = rng.uniform(-100,100,(24,3))
        faces = np.arange(24).reshape(-1,3)
        points = rng.uniform(-130,130,(20,3))
        expected = [brute_force_obj_distance(p,vertices,faces)[0] for p in points]
        actual = compute_surface_deviation(points,vertices,faces)
        np.testing.assert_allclose(actual,expected,atol=1e-10,rtol=0)
        # Duplicates and winding do not change the represented surface distance.
        duplicated = np.concatenate([faces,faces[:,::-1],faces])
        np.testing.assert_allclose(compute_surface_deviation(points,vertices,duplicated),actual,atol=1e-12,rtol=0)

    def test_recorded_audit_points_against_raw_obj_decimal_oracle(self):
        run = ROOT / "data/validation/y_translation_20261006T104604Z_b8c0a39c"
        audit = json.loads((run / "distance_audit.json").read_text())
        vertices, faces = read_obj_triangles(run / "reference/nominal_reference.obj")
        points = np.array([row["engine_max_point_xyz_mm"] for row in audit["cases"]])
        actual = compute_surface_deviation(points,vertices,faces)
        oracle = [brute_force_obj_distance(p,vertices,faces)[0] for p in points]
        np.testing.assert_allclose(actual,oracle,atol=1e-10,rtol=0)
        # Regression of the actual full-mesh false distance, independent of Open3D's version.
        self.assertAlmostEqual(actual[4],0.4315,places=10)
        self.assertAlmostEqual(actual[5],4.4315,places=10)

    def test_rejects_invalid_geometry_and_accepts_empty_queries(self):
        vertices = [[0,0,0],[1,0,0],[0,1,0]]
        for faces in [[], [[0,1,3]], [[-1,1,2]], [[0.,1.,2.]]]:
            with self.assertRaises(ValueError):
                compute_surface_deviation([[0,0,1]],vertices,faces)
        with self.assertRaises(ValueError):
            compute_surface_deviation([[np.nan,0,1]],vertices,[[0,1,2]])
        with self.assertRaises(ValueError):
            compute_surface_deviation([[0,0,1]],[[np.inf,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        self.assertEqual(compute_surface_deviation(np.empty((0,3)),vertices,[[0,1,2]]).shape,(0,))


if __name__ == "__main__":
    unittest.main()
