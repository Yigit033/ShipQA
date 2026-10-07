import json
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import validate_combined_poses as validation


def euler_matrix(degrees):
    rx, ry, rz = np.radians(degrees)
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    mx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    my = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    mz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return mz @ my @ mx


class CombinedPoseScoringTests(unittest.TestCase):
    @staticmethod
    def truth(translation=(0, -8, 4), rotation=(0, 0, 0.75), category=None):
        if category is None:
            category = ("zero_defect_control" if not any(translation + rotation)
                        else "detectable_combined_pose")
        return {
            "units": "mm", "angle_units": "degrees", "category": category,
            "injected_component": "STF_03",
            "injected_translation_xyz_mm": list(translation),
            "required_translation_axes": ["y", "z"],
            "characterization_translation_axes": (["x"] if translation[0] else []),
            "injected_rotation_xyz_deg": list(rotation),
            "rotation_matrix_nominal_to_observed": euler_matrix(rotation).tolist(),
        }

    @staticmethod
    def prediction(translation=(None, -8.1, 4.2), rotation=(0, 0, 0.76),
                   component="STF_03"):
        matrix = euler_matrix(rotation)
        return {
            "status": "estimated", "candidate_count": 100,
            "detected_component": component, "surface_p95_mm": 1.0,
            "surface_p99_mm": 8.0,
            "joint_component_pose": {
                "status": "partial_estimate", "quality_status": "pass_controlled",
                "estimated_translation_xyz_mm": list(translation),
                "estimated_rotation_xyz_deg": list(rotation),
                "rotation_matrix_nominal_to_observed": matrix.tolist(),
                "observed_points_used": 1000,
                "stages": [{"fitness": 0.99, "inlier_rmse_mm": 0.4}],
            },
        }

    def test_required_axes_and_rotation_form_valid_joint_pose(self):
        row = validation.evaluate_case(
            "case_001", self.prediction(), self.truth())
        self.assertEqual(row["outcome"], "estimated")
        self.assertTrue(row["valid_joint_pose"])
        self.assertAlmostEqual(row["signed_error_y_mm"], -0.1)
        self.assertAlmostEqual(row["signed_error_z_mm"], 0.2)
        self.assertIsNone(row["signed_error_x_mm"])

    def test_unobservable_x_is_reported_with_explicit_denominator(self):
        row = validation.evaluate_case(
            "case_001", self.prediction(),
            self.truth(translation=(8, -8, 4)))
        report = validation.aggregate_results([row])
        self.assertTrue(row["valid_joint_pose"])
        self.assertEqual(report["translation"]["x"]["expected_axis_cases"], 1)
        self.assertEqual(report["translation"]["x"]["valid_estimates"], 0)
        self.assertEqual(report["translation"]["x"]["excluded_cases"], ["case_001"])
        self.assertEqual(report["joint_pose"]["valid_joint_pose_estimates"], 1)

    def test_zero_control_no_detection_is_not_false_positive(self):
        truth = self.truth(
            translation=(0, 0, 0), rotation=(0, 0, 0),
            category="zero_defect_control")
        prediction = {"status": "no_candidates", "candidate_count": 0,
                      "detected_component": None,
                      "joint_component_pose": {"status": "not_estimated"}}
        row = validation.evaluate_case("case_001", prediction, truth)
        self.assertEqual(row["outcome"], "expected_no_detection")
        self.assertEqual(validation.aggregate_results([row])[
            "detection"]["zero_control_false_positives"], 0)

    def test_incorrect_component_invalidates_joint_pose(self):
        row = validation.evaluate_case(
            "case_001", self.prediction(component="STF_02"), self.truth())
        self.assertEqual(row["outcome"], "incorrect_component")
        self.assertFalse(row["valid_joint_pose"])

    def test_predictions_finish_before_ground_truth_is_read(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            reference = run / "reference"
            reference.mkdir()
            (reference / "nominal_reference.obj").write_text("reference")
            (reference / "component_manifest.json").write_text("{}")
            case_ids = ["case_001", "case_002"]
            for case_id in case_ids:
                case = run / "cases" / case_id
                (case / "input").mkdir(parents=True)
                (case / "ground_truth").mkdir()
                scan = case / "input" / "scan.xyz"
                scan.write_text("0 0 0\n")
                truth = self.truth()
                truth["scan_sha256"] = validation.file_hash(scan)
                (case / "ground_truth" / "ground_truth.json").write_text(
                    json.dumps(truth))
            metadata = {
                "experiment": "combined_component_rigid_pose",
                "generation_status": "complete", "point_count": 30000,
                "noise_sigma_mm": 0.35, "seed": 42, "case_ids": case_ids,
                "completed_case_ids": case_ids,
                "reference_sha256": {
                    name: validation.file_hash(reference / name)
                    for name in ("nominal_reference.obj", "component_manifest.json")},
            }
            (run / "run_metadata.json").write_text(json.dumps(metadata))
            completed = []
            original_read = validation.read_json

            def guarded_read(path):
                if "ground_truth" in Path(path).parts:
                    self.assertEqual(completed, case_ids)
                return original_read(path)

            def predict(scan, mesh, manifest, output):
                self.assertNotIn("ground_truth", scan.parts)
                output.mkdir(parents=True)
                (output / "prediction.json").write_text(
                    json.dumps(self.prediction()))
                completed.append(output.parent.name)

            output = run / "analysis"
            with mock.patch.object(validation, "read_json", side_effect=guarded_read):
                report = validation.run_validation(
                    run, predictor=predict, output_dir=output)
            self.assertEqual(report["joint_pose"]["expected_detectable_cases"], 2)


class CombinedPoseRhinoContractTests(unittest.TestCase):
    def test_batch_matrix_and_transform_convention(self):
        rhino = types.SimpleNamespace(
            UnitSystem=types.SimpleNamespace(Millimeters="mm"),
            RhinoApp=types.SimpleNamespace(Version="test"))
        with mock.patch.dict(sys.modules, {
                "Rhino": rhino,
                "rhinoscriptsyntax": types.ModuleType("rhinoscriptsyntax"),
                "scriptcontext": types.ModuleType("scriptcontext")}):
            path = ROOT / "rhino_scripts" / "09_generate_combined_pose_validation_batch.py"
            spec = importlib.util.spec_from_file_location("combined_pose_rhino", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        self.assertEqual(len(module.POSES), 11)
        self.assertEqual(module.POSES[0], ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        translation = (4.0, -8.0, 3.0)
        rotation = module.euler_xyz_matrix((2.0, -0.5, 0.75))
        transform = np.asarray(module.rigid_transform(
            rotation, module.ROTATION_CENTER_MM, translation))
        center = np.asarray(list(module.ROTATION_CENTER_MM) + [1.0])
        np.testing.assert_allclose(
            (transform @ center)[:3],
            np.asarray(module.ROTATION_CENTER_MM) + translation, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
