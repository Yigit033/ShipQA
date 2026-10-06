import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import component_pose
import validate_xyz_translations as validation


class ComponentPoseTests(unittest.TestCase):
    def test_grouped_obj_selects_only_requested_parts(self):
        obj = """\
g STF_03_WEB
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
g STF_04_WEB
v 10 0 0
v 11 0 0
v 10 1 0
f 4 5 6
"""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "reference.obj"
            path.write_text(obj)
            vertices, triangles = component_pose.load_grouped_obj_component(
                path, ["STF_03_WEB"])
        np.testing.assert_allclose(vertices, [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        np.testing.assert_array_equal(triangles, [[0, 1, 2]])

    def test_group_identity_is_required_for_xyz_pose(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "reference.obj"
            path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
            with self.assertRaisesRegex(ValueError, "grouped faces"):
                component_pose.load_grouped_obj_component(path, ["STF_03_WEB"])

    def test_estimator_does_not_receive_ground_truth_or_changed_axis(self):
        parameters = component_pose.estimate_translation_xyz.__code__.co_varnames[
            :component_pose.estimate_translation_xyz.__code__.co_argcount]
        self.assertEqual(parameters, (
            "scan_points", "component", "mesh_path", "legacy_y_shift_mm",
            "legacy_component_points"))

    def test_known_component_surface_translation_is_recovered(self):
        obj = """\
g STF_03_WEB
v 0 0 10
v 100 0 10
v 0 20 10
v 0 0 30
f 1 2 3
f 1 2 4
f 1 3 4
f 2 3 4
"""
        component = {"part_ids": ["STF_03_WEB"],
                     "bbox": {"min": [0, 0, 10], "max": [100, 20, 30]}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "reference.obj"
            path.write_text(obj)
            vertices, triangles = component_pose.load_grouped_obj_component(
                path, component["part_ids"])
            nominal = component_pose.sample_triangle_mesh(vertices, triangles)
            translated = nominal + np.array([8.0, -4.0, 12.0])
            result = component_pose.estimate_translation_xyz(
                translated, component, path, -4.0, translated)
        np.testing.assert_allclose(
            result["estimated_translation_xyz_mm"], [8.0, -4.0, 12.0], atol=1e-12)


class XYZScoringTests(unittest.TestCase):
    @staticmethod
    def truth(vector):
        category = "zero_defect_control" if vector == [0, 0, 0] else "detectable_translation"
        return {"units": "mm", "category": category,
                "injected_component": "STF_03",
                "injected_translation_xyz_mm": vector}

    @staticmethod
    def prediction(vector=None, component="STF_03"):
        if vector is None:
            return {"status": "no_candidates", "candidate_count": 0,
                    "detected_component": None, "estimated_translation_xyz_mm": None,
                    "points_used": 0, "component_pose": {}}
        return {"status": "estimated", "candidate_count": 20,
                "detected_component": component, "estimated_translation_xyz_mm": vector,
                "points_used": 100, "component_pose": {"pose_points_used": 120}}

    def test_axis_and_vector_errors(self):
        row = validation.evaluate_case(
            "case_001", self.prediction([8.5, -3.0, 1.0]),
            self.truth([8.0, -4.0, 0.0]))
        self.assertEqual(row["signed_error_x_mm"], 0.5)
        self.assertEqual(row["signed_error_y_mm"], 1.0)
        self.assertEqual(row["signed_error_z_mm"], 1.0)
        self.assertAlmostEqual(row["vector_error_mm"], 1.5)

    def test_zero_control_no_detection_is_correct(self):
        row = validation.evaluate_case(
            "case_001", self.prediction(), self.truth([0, 0, 0]))
        self.assertEqual(row["outcome"], "expected_no_detection")
        self.assertFalse(row["valid_estimate"])
        report = validation.aggregate_results([row])
        self.assertEqual(report["detection"]["zero_control_false_positives"], 0)

    def test_failure_denominator_is_explicit(self):
        rows = [
            validation.evaluate_case("case_001", self.prediction([8, 0, 0]),
                                     self.truth([8, 0, 0])),
            validation.evaluate_case("case_002", self.prediction(),
                                     self.truth([-8, 0, 0])),
            validation.evaluate_case("case_003", self.prediction([0, 8, 0], "STF_02"),
                                     self.truth([0, 8, 0])),
        ]
        report = validation.aggregate_results(rows)
        self.assertEqual(report["estimation"]["expected_detectable_cases"], 3)
        self.assertEqual(report["estimation"]["valid_estimates"], 1)
        self.assertEqual(len(report["estimation"]["excluded_cases"]), 2)

    def test_predictions_finish_before_truth_is_read(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "reference").mkdir()
            (root / "reference" / "nominal_reference.obj").write_text("reference")
            (root / "reference" / "component_manifest.json").write_text("{}")
            case_ids = ["case_001", "case_002"]
            for case_id, vector in zip(case_ids, ([8, 0, 0], [0, 0, 0])):
                case = root / "cases" / case_id
                (case / "input").mkdir(parents=True)
                (case / "ground_truth").mkdir()
                scan = case / "input" / "scan.xyz"
                scan.write_text("0 0 0\n")
                truth = self.truth(vector)
                truth["scan_sha256"] = validation.file_hash(scan)
                (case / "ground_truth" / "ground_truth.json").write_text(json.dumps(truth))
            metadata = {
                "experiment": "independent_xyz_component_translation",
                "generation_status": "complete", "units": "mm",
                "point_count": 30000, "noise_sigma_mm": 0.35, "seed": 42,
                "case_ids": case_ids, "completed_case_ids": case_ids,
                "reference_sha256": {
                    name: validation.file_hash(root / "reference" / name)
                    for name in ("nominal_reference.obj", "component_manifest.json")},
            }
            (root / "run_metadata.json").write_text(json.dumps(metadata))
            completed = []
            original = validation.read_json

            def guarded_read(path):
                if "ground_truth" in Path(path).parts:
                    self.assertEqual(completed, case_ids)
                return original(path)

            def predict(scan, mesh, manifest, output):
                self.assertNotIn("ground_truth", scan.parts)
                output.mkdir(parents=True)
                value = self.prediction()
                (output / "prediction.json").write_text(json.dumps(value))
                completed.append(output.parent.name)

            with mock.patch.object(validation, "read_json", side_effect=guarded_read):
                report = validation.run_validation(root, predictor=predict)
            self.assertEqual(report["estimation"]["expected_detectable_cases"], 1)


if __name__ == "__main__":
    unittest.main()
