import inspect
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
import open3d as o3d


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import generate_registration_validation as generator
import qa_engine
import registration
import validate_registration
import validate_y_translations as validation


SWEEP = ROOT / "data/validation/y_translation_20261006T104604Z_b8c0a39c"


class RegistrationTests(unittest.TestCase):
    def test_transform_convention_and_inverse(self):
        transform = generator.euler_xyz_transform(
            1.2, -0.8, 1.5, [75, -45, 30], [2000, 1200, 86])
        points = np.array([[0.0, 0.0, 0.0], [2000.0, 1200.0, 86.0],
                           [4000.0, 2400.0, 172.0]])
        moved = registration.apply_transform(points, transform)
        restored = registration.apply_transform(moved, np.linalg.inv(transform))
        np.testing.assert_allclose(restored, points, atol=1e-12)
        np.testing.assert_allclose(moved[1], points[1] + [75, -45, 30], atol=1e-12)

    def test_controlled_rigid_offset_preserves_local_defect(self):
        scan = np.loadtxt(SWEEP / "cases/case_003/input/scan.xyz")
        transform = generator.euler_xyz_transform(
            1.2, -0.8, 1.5, [75, -45, 30], [2000, 1200, 86])
        moved = registration.apply_transform(scan, transform)
        mesh_path = SWEEP / "reference/nominal_reference.obj"
        manifest_path = SWEEP / "reference/component_manifest.json"
        baseline_shift_y_mm = -7.617638
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            scan_path = folder / "scan.xyz"
            np.savetxt(scan_path, moved, fmt="%.10f")
            result = qa_engine.run_prediction(
                scan_path, mesh_path, manifest_path, folder / "prediction", registration=True)
            self.assertEqual(result["status"], "estimated")
            self.assertEqual(result["registration"]["quality_status"], "pass_provisional")
            self.assertEqual(result["detected_component"], "STF_03")
            self.assertLess(abs(result["estimated_shift_y_mm"]
                                - baseline_shift_y_mm), 0.25)
            self.assertTrue((folder / "prediction/registered_scan.xyz").is_file())

            estimated = np.asarray(result["registration"]["transform_scan_to_cad"])
            expected = np.linalg.inv(transform)
            rotation_delta = estimated[:3, :3] @ expected[:3, :3].T
            self.assertLess(registration.rotation_angle_degrees(rotation_delta), 0.01)
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            vertices = np.asarray(mesh.vertices)
            round_trip = estimated @ transform
            residual = registration.apply_transform(vertices, round_trip) - vertices
            self.assertLess(float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))), 0.5)
            self.assertLess(result["registration"]["post_surface_p95_mm"],
                            result["registration"]["pre_surface_p95_mm"] * 0.05)

    def test_prediction_interfaces_cannot_receive_ground_truth(self):
        self.assertEqual(list(inspect.signature(registration.register_scan_to_nominal).parameters),
                         ["scan_points", "vertices", "triangles", "initial_transform",
                          "filter_isolated_outliers"])
        self.assertEqual(list(inspect.signature(qa_engine.run_prediction).parameters),
                         ["scan_path", "mesh_path", "manifest_path", "output_dir", "registration",
                          "initial_alignment_path", "filter_isolated_outliers"])
        for path in (ROOT / "src/registration.py", ROOT / "src/qa_engine.py"):
            source = path.read_text(encoding="utf-8")
            for token in ("KNOWN_SHIFT", "KNOWN_DEFECT", "injected_shift", "STF_03"):
                self.assertNotIn(token, source)

    def test_field_coarse_alignment_inputs(self):
        cad = np.array([[0.0, 0.0, 0.0], [4000.0, 0.0, 0.0],
                        [0.0, 2400.0, 0.0], [4000.0, 2400.0, 172.0]])
        forward = generator.euler_xyz_transform(
            8.0, -5.0, 12.0, [300, -200, 100], [2000, 1200, 86])
        scan = registration.apply_transform(cad, forward)
        estimated, details = registration.initial_transform_from_spec({
            "units": "mm", "method": "corresponding_points",
            "source": "unit_test", "scan_points_mm": scan.tolist(),
            "cad_points_mm": cad.tolist(),
        })
        np.testing.assert_allclose(estimated, np.linalg.inv(forward), atol=1e-10)
        self.assertLess(details["correspondence_rmse_mm"], 1e-10)
        matrix, details = registration.initial_transform_from_spec({
            "units": "mm", "method": "transform_matrix",
            "source": "scanner_pose", "transform_scan_to_cad": np.linalg.inv(forward).tolist(),
        })
        np.testing.assert_allclose(matrix, np.linalg.inv(forward), atol=1e-12)
        self.assertEqual(details["source"], "scanner_pose")
        with self.assertRaisesRegex(ValueError, "collinear"):
            registration.rigid_transform_from_correspondences(
                [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
                [[0, 1, 0], [1, 1, 0], [2, 1, 0]])

    def test_isolated_outlier_filter(self):
        grid = np.array([[x, y, 0.0] for x in range(20) for y in range(20)], dtype=float)
        outliers = np.array([[1000.0 + 100 * i, -1000.0 - 50 * i, 500.0] for i in range(20)])
        filtered, retained = registration.remove_isolated_outliers(np.vstack([grid, outliers]))
        self.assertGreater(len(filtered), 350)
        self.assertFalse(np.any(retained >= len(grid)))

    def test_failed_registration_blocks_component_qa(self):
        mesh_path = SWEEP / "reference/nominal_reference.obj"
        manifest_path = SWEEP / "reference/component_manifest.json"
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            scan_path = folder / "scan.xyz"
            np.savetxt(scan_path, [[100, 100, 12], [200, 200, 12], [300, 300, 12]])
            diagnostics = {
                "status": "completed", "transform_scan_to_cad": np.eye(4).tolist(),
                "stages": [{"fitness": 0.2, "max_correspondence_mm": 15.0}],
                "retained_point_fraction": 1.0,
            }
            with mock.patch.object(
                    qa_engine, "register_scan_to_nominal",
                    return_value=(np.loadtxt(scan_path), np.arange(3), diagnostics)):
                result = qa_engine.run_prediction(
                    scan_path, mesh_path, manifest_path, folder / "prediction",
                    registration=True)
            self.assertEqual(result["status"], "registration_rejected")
            self.assertIsNone(result["candidate_count"])
            self.assertIsNone(result["detected_component"])
            self.assertIsNone(result["estimated_shift_y_mm"])
            self.assertEqual(result["registration"]["quality_status"], "fail")


class RegistrationIsolationTests(unittest.TestCase):
    def test_all_registered_predictions_finish_before_truth_is_read(self):
        reference_mesh = o3d.io.read_triangle_mesh(
            str(SWEEP / "reference/nominal_reference.obj"))
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            run = folder / "run"
            (run / "reference").mkdir(parents=True)
            for name in ("nominal_reference.obj", "component_manifest.json"):
                (run / "reference" / name).write_bytes((SWEEP / "reference" / name).read_bytes())
            case_ids = ["case_001", "case_002"]
            identity = np.eye(4).tolist()
            for index, case_id in enumerate(case_ids):
                case = run / "cases" / case_id
                (case / "input").mkdir(parents=True)
                (case / "ground_truth").mkdir()
                scan = case / "input/scan.xyz"
                scan.write_text("100 100 12\n200 200 12\n300 300 12\n")
                truth = {
                    "units": "mm", "injected_shift_y_mm": 0 if index else 8,
                    "category": "zero_defect_control" if index else "detectable_translation",
                    "injected_component": "STF_03", "scan_sha256": validation.file_hash(scan),
                    "global_transform_cad_to_scan": identity,
                }
                validation.write_json_new(case / "ground_truth/ground_truth.json", truth)
            validation.write_json_new(run / "run_metadata.json", {
                "experiment": "controlled_global_rigid_registration",
                "generation_status": "complete", "case_ids": case_ids,
                "completed_case_ids": case_ids,
                "reference_sha256": {name: validation.file_hash(run / "reference" / name)
                                     for name in ("nominal_reference.obj", "component_manifest.json")},
            })

            completed = []
            original_read = validate_registration.read_json

            def guarded_read(path):
                if "ground_truth" in Path(path).parts:
                    self.assertEqual(completed, case_ids)
                return original_read(path)

            def predictor(scan, mesh, manifest, output):
                for path in (scan, mesh, manifest, output):
                    self.assertNotIn("ground_truth", Path(path).parts)
                output.mkdir(parents=True)
                prediction = {
                    "status": "no_candidates", "units": "mm", "scan_point_count": 3,
                    "candidate_count": 0, "detected_component": None,
                    "estimated_shift_y_mm": None, "points_used": 0,
                    "surface_p95_mm": 0.0, "surface_p99_mm": 0.0,
                    "input_sha256": {"scan": validation.file_hash(scan),
                                     "mesh": validation.file_hash(mesh),
                                     "manifest": validation.file_hash(manifest)},
                    "registration": {"status": "completed", "transform_scan_to_cad": identity,
                                     "pre_surface_p95_mm": 0.0, "post_surface_p95_mm": 0.0,
                                     "stages": [{"fitness": 1.0}]},
                }
                validation.write_json_new(output / "prediction.json", prediction)
                completed.append(output.parent.name)

            with mock.patch.object(validate_registration, "read_json", side_effect=guarded_read), \
                    mock.patch.object(validate_registration.o3d.io, "read_triangle_mesh",
                                      return_value=reference_mesh):
                aggregate = validate_registration.run_validation(
                    run, folder / "analysis", predictor=predictor)
            self.assertEqual(aggregate["registration"]["completed_cases"], 2)
            self.assertEqual(aggregate["downstream_qa"]["estimation"]["valid_estimates"], 0)


if __name__ == "__main__":
    unittest.main()
