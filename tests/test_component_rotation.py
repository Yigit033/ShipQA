import inspect
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import component_pose
import component_rotation
import validate_component_rotations as validation


def euler_matrix(values):
    rx, ry, rz = np.radians(values)
    mx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    my = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    mz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    return mz @ my @ mx


class RotationPrimitiveTests(unittest.TestCase):
    def test_euler_round_trip_and_sign_convention(self):
        expected = [1.2, -0.8, 1.5]
        actual = component_rotation.rotation_matrix_to_euler_xyz_degrees(
            euler_matrix(expected))
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_estimator_has_no_ground_truth_or_expected_axis_input(self):
        self.assertEqual(list(inspect.signature(
            component_rotation.estimate_component_rotation).parameters),
            ["scan_points", "component", "mesh_path"])
        source = (ROOT / "src/component_rotation.py").read_text()
        for token in ("ground_truth", "KNOWN_ROTATION", "injected_rotation", "STF_03"):
            self.assertNotIn(token, source)

    def test_known_local_rotation_is_recovered(self):
        obj = """\
g PART_A
v 0 0 20
v 100 0 20
v 0 30 20
v 0 0 70
f 1 2 3
f 1 2 4
f 1 3 4
f 2 3 4
"""
        component = {"part_ids": ["PART_A"],
                     "bbox": {"min": [-100, -100, 0], "max": [200, 150, 100]},
                     "nominal_center": [25, 7.5, 32.5]}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "reference.obj"
            path.write_text(obj)
            vertices, triangles = component_pose.load_grouped_obj_component(path, ["PART_A"])
            nominal = component_pose.sample_triangle_mesh(vertices, triangles, count=20000)
            expected_angles = np.array([1.0, -0.6, 0.8])
            rotation = euler_matrix(expected_angles)
            center = np.asarray(component["nominal_center"])
            observed = (nominal - center) @ rotation.T + center
            result = component_rotation.estimate_component_rotation(
                observed, component, path)
        self.assertEqual(result["status"], "estimated_provisional")
        self.assertLess(validation.rotation_error_degrees(
            result["rotation_matrix_nominal_to_observed"], rotation), 0.1)


class RotationScoringTests(unittest.TestCase):
    def test_angular_error_and_wrong_component(self):
        truth = {"units": "mm", "angle_units": "degrees",
                 "category": "detectable_rotation", "injected_component": "STF_03",
                 "injected_rotation_xyz_deg": [0, 0.75, 0],
                 "rotation_matrix_nominal_to_observed": euler_matrix([0, 0.75, 0]).tolist()}
        prediction = {"status": "estimated", "candidate_count": 100,
                      "detected_component": "STF_03", "surface_p95_mm": 1,
                      "surface_p99_mm": 7,
                      "component_rotation": {"status": "estimated_provisional",
                          "rotation_matrix_nominal_to_observed": euler_matrix([0, 0.8, 0]).tolist(),
                          "estimated_rotation_xyz_deg": [0, 0.8, 0],
                          "observed_points_used": 1000,
                          "stages": [{"fitness": 0.9, "inlier_rmse_mm": 0.4}]}}
        row = validation.evaluate_case("neutral", prediction, truth)
        self.assertEqual(row["outcome"], "estimated")
        self.assertAlmostEqual(row["angular_error_deg"], 0.05, places=8)
        prediction["detected_component"] = "STF_02"
        row = validation.evaluate_case("neutral", prediction, truth)
        self.assertEqual(row["outcome"], "incorrect_component")
        self.assertFalse(row["valid_rotation_estimate"])

    def test_zero_control_no_detection_is_not_false_positive(self):
        truth = {"units": "mm", "angle_units": "degrees",
                 "category": "zero_defect_control", "injected_component": "STF_03",
                 "injected_rotation_xyz_deg": [0, 0, 0],
                 "rotation_matrix_nominal_to_observed": np.eye(3).tolist()}
        prediction = {"status": "no_candidates", "candidate_count": 0,
                      "detected_component": None, "component_rotation": {}}
        row = validation.evaluate_case("neutral", prediction, truth)
        self.assertEqual(row["outcome"], "expected_no_detection")


if __name__ == "__main__":
    unittest.main()
