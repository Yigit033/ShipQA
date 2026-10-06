import ast
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import qa_engine
import validate_y_translations as validation
from distance_reference import box_surface_distances

REGRESSION = ROOT / "data" / "regression" / "stf03_negative_8"


def truth(shift=-8.0, component="STF_03"):
    return {"units": "mm", "injected_shift_y_mm": shift, "injected_component": component,
            "category": validation.category_for_shift(shift)}


def prediction(estimate=-8.126176, component="STF_03", candidates=627):
    return {"status": "estimated", "candidate_count": candidates,
            "detected_component": component, "estimated_shift_y_mm": estimate,
            "points_used": 1945, "surface_p95_mm": 0.78, "surface_p99_mm": 7.96}


def no_candidates():
    return {"status": "no_candidates", "candidate_count": 0,
            "detected_component": None, "estimated_shift_y_mm": None, "points_used": 0,
            "surface_p95_mm": 0.7, "surface_p99_mm": 0.9}


class ScoringTests(unittest.TestCase):
    def test_signed_and_absolute_errors_both_directions(self):
        for injected, estimated, signed in [(-8, -8.126176, -0.126176), (12, 11.874, -0.126), (8, 8.2, 0.2)]:
            row = validation.evaluate_case("neutral", prediction(estimated), truth(injected))
            self.assertAlmostEqual(row["signed_error_mm"], signed)
            self.assertAlmostEqual(row["absolute_error_mm"], abs(signed))
            self.assertTrue(row["eligible_for_estimation_metrics"])

    def test_zero_control_without_estimate_is_correct(self):
        row = validation.evaluate_case("neutral", no_candidates(), truth(0))
        self.assertEqual(row["outcome"], "expected_no_detection")
        self.assertFalse(row["false_positive_component"])
        self.assertIsNone(row["relative_absolute_error_percent"])
        report = validation.aggregate_results([row])
        self.assertEqual(report["zero_defect_control"]["correct_no_detection_cases"], 1)
        self.assertEqual(report["estimation"]["expected_detectable_cases"], 0)
        self.assertIsNone(report["estimation"]["mae_mm"])

    def test_zero_control_false_positive(self):
        row = validation.evaluate_case("neutral", prediction(0.1), truth(0))
        self.assertEqual(row["outcome"], "false_positive")
        self.assertTrue(row["false_positive_component"])
        self.assertFalse(row["eligible_for_estimation_metrics"])

    def test_subthreshold_missing_estimate_is_characterization(self):
        rows = [validation.evaluate_case("neutral", no_candidates(), truth(s)) for s in (-4, 4)]
        self.assertTrue(all(r["outcome"] == "characterized" for r in rows))
        report = validation.aggregate_results(rows)
        self.assertEqual(report["sub_threshold_characterization"]["no_detection_cases"], 2)
        self.assertEqual(report["sub_threshold_characterization"]["prediction_failure_cases"], 0)
        self.assertEqual(report["estimation"]["expected_detectable_cases"], 0)

    def test_subthreshold_estimate_not_in_detectable_mae(self):
        row = validation.evaluate_case("neutral", prediction(3.9), truth(4))
        self.assertTrue(row["valid_estimate"])
        self.assertFalse(row["eligible_for_estimation_metrics"])

    def test_missing_prediction_is_not_no_detection(self):
        for shift in (0, 4, 8):
            row = validation.evaluate_case("neutral", None, truth(shift))
            self.assertEqual(row["outcome"], "missing_prediction")
            self.assertIsNone(row["detected"])
            self.assertIsNone(row["signed_error_mm"])

    def test_detectable_no_detection_is_reported(self):
        row = validation.evaluate_case("neutral", no_candidates(), truth(8))
        self.assertEqual(row["outcome"], "no_detection")
        report = validation.aggregate_results([row])
        self.assertEqual(report["detectable_translation_detection"]["no_detection_cases"], 1)
        self.assertEqual(report["estimation"]["valid_estimates"], 0)
        self.assertEqual(report["estimation"]["expected_detectable_cases"], 1)

    def test_wrong_component_not_scored_as_target(self):
        row = validation.evaluate_case("neutral", prediction(component="STF_02"), truth())
        self.assertEqual(row["outcome"], "incorrect_component")
        self.assertIsNone(row["absolute_error_mm"])
        self.assertFalse(row["eligible_for_estimation_metrics"])

    def test_incomplete_estimation_after_correct_identification(self):
        p = prediction()
        p.update(status="no_component_points", points_used=0, estimated_shift_y_mm=None)
        row = validation.evaluate_case("neutral", p, truth())
        self.assertTrue(row["correct_component"])
        self.assertEqual(row["outcome"], "missing_estimate")

    def test_denominator_and_bias_expose_failures(self):
        rows = [validation.evaluate_case("case_001", prediction(), truth()),
                validation.evaluate_case("case_002", None, truth(8)),
                validation.evaluate_case("case_003", prediction(12.2), truth(12)),
                validation.evaluate_case("case_004", prediction(-16, "STF_02"), truth(-16))]
        report = validation.aggregate_results(rows)
        self.assertEqual(report["estimation"]["expected_detectable_cases"], 4)
        self.assertEqual(report["estimation"]["valid_estimates"], 2)
        self.assertEqual(len(report["estimation"]["excluded_cases"]), 2)
        self.assertAlmostEqual(report["estimation"]["mean_signed_error_mm"], (-0.126176 + 0.2) / 2)
        self.assertAlmostEqual(report["estimation"]["mae_mm"], (0.126176 + 0.2) / 2)
        self.assertAlmostEqual(report["estimation"]["maximum_absolute_error_mm"], 0.2)

    def test_nonfinite_prediction_cannot_enter_metrics(self):
        row = validation.evaluate_case("neutral", prediction(float("nan")), truth())
        self.assertEqual(row["outcome"], "invalid_prediction")
        self.assertFalse(row["eligible_for_estimation_metrics"])

    def test_truth_never_inferred_from_case_name(self):
        row = validation.evaluate_case("STF_03_plus_12", prediction(8.2, "STF_04"), truth(8, "STF_04"))
        self.assertAlmostEqual(row["signed_error_mm"], 0.2)
        with self.assertRaises(KeyError):
            validation.evaluate_case("STF_03_minus_8", prediction(), {"units": "mm"})


class EngineTests(unittest.TestCase):
    def test_archived_negative_8_regression(self):
        checksums = validation.read_json(REGRESSION / "checksums.json")
        for name, expected in checksums.items():
            self.assertEqual(validation.file_hash(REGRESSION / name), expected)
        baseline = validation.read_json(REGRESSION / "baseline" / "result.json")
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "prediction"
            actual = qa_engine.run_prediction(
                REGRESSION / "input" / "scan.xyz", REGRESSION / "reference" / "nominal_reference.obj",
                REGRESSION / "reference" / "component_manifest.json", out)
            self.assertEqual(actual["status"], "estimated")
            for key in ("detected_component", "points_used"):
                self.assertEqual(actual[key], baseline[key])
            self.assertAlmostEqual(actual["estimated_shift_y_mm"], -8.126176, places=8)
            self.assertAlmostEqual(actual["observed_center_y_mm"], baseline["observed_center_y_mm"], places=7)
            # The old surface statistics are historical evidence of the float32 bug,
            # not correct reference distances. Check against the known nominal boxes.
            metadata = validation.read_json(ROOT / "data/validation/y_translation_20261006T104604Z_b8c0a39c/run_metadata.json")
            points = np.loadtxt(REGRESSION / "input/scan.xyz")
            oracle = box_surface_distances(points, [p["bbox_mm"] for p in metadata["nominal_parts_in_sampling_order"]])
            for percentile in (95,99):
                self.assertAlmostEqual(actual["surface_p%d_mm" % percentile], float(np.percentile(oracle,percentile)), places=9)
            self.assertEqual(actual["candidate_count"], int((oracle>5).sum()))
            np.testing.assert_array_equal(np.loadtxt(out / "defect_candidates.xyz"),points[oracle>5])
            with self.assertRaises(FileExistsError):
                qa_engine.run_prediction(None, None, None, out)

    def test_actual_engine_no_candidates_outputs_null_estimate(self):
        # Points exactly on the reference plate; a unit fixture, not a sweep scan.
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            scan = folder / "scan.xyz"
            np.savetxt(scan, [[100, 100, 12], [200, 200, 12]])
            result = qa_engine.run_prediction(scan, REGRESSION / "reference" / "nominal_reference.obj",
                                              REGRESSION / "reference" / "component_manifest.json", folder / "out")
            self.assertEqual(result["status"], "no_candidates")
            self.assertIsNone(result["estimated_shift_y_mm"])
            self.assertIsNone(result["detected_component"])
            self.assertEqual((folder / "out" / "defect_candidates.xyz").stat().st_size, 0)

    def test_single_threshold_outlier_has_no_engineering_support(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            scan = folder / "scan.xyz"
            np.savetxt(scan, [[100, 100, 12], [200, 200, 12], [300, 300, 18]])
            result = qa_engine.run_prediction(
                scan, REGRESSION / "reference" / "nominal_reference.obj",
                REGRESSION / "reference" / "component_manifest.json", folder / "out")
            self.assertEqual(result["status"], "no_candidates")
            self.assertEqual(result["raw_candidate_count"], 1)
            self.assertEqual(result["candidate_count"], 0)
            self.assertEqual(result["rejected_sparse_candidate_count"], 1)
            self.assertIsNone(result["detected_component"])

    def test_invalid_scan_is_execution_error(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            scan = folder / "scan.xyz"
            scan.write_text("nan 0 0\n1 2 3\n")
            result = qa_engine.run_prediction(scan, REGRESSION / "reference" / "nominal_reference.obj",
                                              REGRESSION / "reference" / "component_manifest.json", folder / "out")
            self.assertEqual(result["status"], "error")

    def test_numerical_assignments_unchanged(self):
        old = ast.parse((REGRESSION / "provenance" / "qa_engine_before.py.txt").read_text())
        new = ast.parse((ROOT / "src" / "qa_engine.py").read_text())
        names = ("DEFECT_THRESHOLD_MM", "X_MARGIN", "Y_MARGIN", "Z_MARGIN", "MIN_COMPONENT_Z",
                 "component_mask", "y_low", "y_high", "observed_center_y", "estimated_shift_y",
                 "defect_mask", "defect_center_y", "distance_y")
        def expressions(tree):
            return {n.targets[0].id: ast.dump(n.value) for n in ast.walk(tree)
                    if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                    and n.targets[0].id in names}
        self.assertEqual(expressions(old), expressions(new))


class IsolationTests(unittest.TestCase):
    def make_run(self, root):
        (root / "reference").mkdir()
        for name in ("nominal_reference.obj", "component_manifest.json"):
            shutil.copyfile(REGRESSION / "reference" / name, root / "reference" / name)
        ids = ["case_001", "case_002"]
        for case_id in ids:
            case = root / "cases" / case_id
            (case / "input").mkdir(parents=True)
            (case / "input" / "scan.xyz").write_text("100 100 12\n200 200 12\n")
            (case / "ground_truth").mkdir()
            t = truth(8 if case_id == "case_001" else 0)
            t["scan_sha256"] = validation.file_hash(case / "input" / "scan.xyz")
            validation.write_json_new(case / "ground_truth" / "ground_truth.json", t)
        validation.write_json_new(root / "run_metadata.json", {
            "generation_status": "complete", "case_ids": ids, "completed_case_ids": ids,
            "units": "mm", "point_count": 30000, "noise_sigma_mm": 0.35, "seed": 42,
            "reference_sha256": {p.name: validation.file_hash(p) for p in (root / "reference").iterdir()},
        })
        return ids

    def test_all_predictions_finish_before_any_ground_truth_read(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ids = self.make_run(root)
            predicted = []
            original_read = validation.read_json
            def guarded_read(path):
                if "ground_truth" in Path(path).parts:
                    self.assertEqual(predicted, ids)
                return original_read(path)
            def predict(scan, mesh, manifest, output_dir):
                for path in (scan, mesh, manifest, output_dir):
                    self.assertNotIn("ground_truth", path.parts)
                output_dir.mkdir()
                result = no_candidates()
                result["scan_point_count"] = 30000
                result["input_sha256"] = {"scan": validation.file_hash(scan), "mesh": validation.file_hash(mesh),
                                          "manifest": validation.file_hash(manifest)}
                validation.write_json_new(output_dir / "prediction.json", result)
                predicted.append(output_dir.parent.name)
            with mock.patch.object(validation, "read_json", side_effect=guarded_read):
                report = validation.run_validation(root, predictor=predict)
            self.assertEqual(report["detectable_translation_detection"]["no_detection_cases"], 1)
            self.assertEqual(report["zero_defect_control"]["correct_no_detection_cases"], 1)
            with self.assertRaises(FileExistsError):
                validation.run_validation(root, predictor=predict)

    def test_engine_command_only_receives_permitted_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            out = folder / "prediction"
            args = [folder / "input" / "scan.xyz", folder / "reference.obj", folder / "manifest.json", out]
            with mock.patch.object(validation.subprocess, "run") as process:
                process.return_value = types.SimpleNamespace(returncode=1, stdout="", stderr="startup error")
                validation.predict_case(*args)
                command = process.call_args.args[0]
                self.assertEqual(command[3:], ["--scan", str(args[0]), "--mesh", str(args[1]),
                                              "--manifest", str(args[2]), "--output-dir", str(out)])
            self.assertEqual(validation.read_json(out / "prediction.json")["status"], "error")

    def test_separate_analysis_directory_preserves_source_and_isolation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "source"
            root.mkdir()
            ids = self.make_run(root)
            before = {str(p.relative_to(root)):p.read_bytes() for p in root.rglob("*") if p.is_file()}
            output = Path(folder) / "analysis"
            completed = []
            original_read = validation.read_json
            def guarded_read(path):
                if "ground_truth" in Path(path).parts:
                    self.assertEqual(completed,ids)
                return original_read(path)
            def predict(scan,mesh,manifest,destination):
                self.assertTrue(destination.is_relative_to(output.resolve()))
                self.assertTrue(scan.is_relative_to(root.resolve()))
                destination.mkdir(parents=True)
                p = no_candidates()
                p.update(scan_point_count=30000,input_sha256={"scan":validation.file_hash(scan),
                         "mesh":validation.file_hash(mesh),"manifest":validation.file_hash(manifest)})
                validation.write_json_new(destination / "prediction.json",p)
                completed.append(destination.parent.name)
            with mock.patch.object(validation,"read_json",side_effect=guarded_read):
                validation.run_validation(root,predictor=predict,output_dir=output)
            self.assertTrue((output / "aggregate.json").is_file())
            self.assertEqual(before,{str(p.relative_to(root)):p.read_bytes() for p in root.rglob("*") if p.is_file()})
            with self.assertRaises(FileExistsError):
                validation.run_validation(root,predictor=predict,output_dir=output)

    def test_engine_source_has_no_experiment_label_access(self):
        source = (ROOT / "src" / "qa_engine.py").read_text()
        for token in ("ground_truth", "KNOWN_SHIFT", "KNOWN_DEFECT", "injected_shift", "STF_03"):
            self.assertNotIn(token, source)


def load_rhino_script(name):
    spec = importlib.util.spec_from_file_location("rhino_test", ROOT / "rhino_scripts" / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RhinoInterfaceTests(unittest.TestCase):
    def test_sampler_uses_geometry_independently_of_reported_shift(self):
        class Point:
            def __init__(self, x, y, z):
                self.X, self.Y, self.Z = x, y, z
            def __sub__(self, other):
                return Point(self.X - other.X, self.Y - other.Y, self.Z - other.Z)
            @property
            def Length(self):
                return (self.X ** 2 + self.Y ** 2 + self.Z ** 2) ** 0.5
        class Vector:
            @staticmethod
            def CrossProduct(a, b):
                return Point(a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X)
        class FaceList(list):
            @property
            def Count(self):
                return len(self)
        class Mesh:
            Vertices = [Point(0, 0, 0), Point(10, 0, 0), Point(0, 10, 0)]
            Faces = FaceList([types.SimpleNamespace(IsTriangle=True, A=0, B=1, C=2)])
        class Cloud:
            def Add(self, point):
                pass
        rhino = types.SimpleNamespace(Geometry=types.SimpleNamespace(
            Point3d=Point, Vector3d=Vector, Mesh=Mesh, Brep=type("Brep", (), {}), PointCloud=Cloud,
            MeshingParameters=types.SimpleNamespace(FastRenderMesh=None)))
        shift_text = ["-8"]
        rs = types.SimpleNamespace(IsLayer=lambda name: True,
                                   GetUserText=lambda obj, key: {"PART_ID": "STF_03_WEB", "ASSEMBLY_ID": "STF_03",
                                                                 "KNOWN_SHIFT_Y_MM": shift_text[0]}[key])
        sc = types.SimpleNamespace(doc=types.SimpleNamespace(Objects=types.SimpleNamespace(
            Find=lambda obj: types.SimpleNamespace(Geometry=Mesh()))))
        with mock.patch.dict(sys.modules, {"Rhino": rhino, "rhinoscriptsyntax": rs, "scriptcontext": sc}):
            module = load_rhino_script("03_generate_synthetic_scan.py")
            with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()):
                first, second = Path(folder) / "first.xyz", Path(folder) / "second.xyz"
                result = module.generate_synthetic_scan(["web"], str(first), display=False, overwrite=False)
                shift_text[0] = "99999"
                module.generate_synthetic_scan(["web"], str(second), display=False, overwrite=False)
                self.assertEqual(result["point_count"], 30000)
                self.assertEqual(first.read_bytes(), second.read_bytes())
                with self.assertRaisesRegex(RuntimeError, "overwrite"):
                    module.generate_synthetic_scan(["web"], str(first), display=False, overwrite=False)

    def test_batch_orchestration_with_mock_geometry(self):
        # Tests folder layout and call sequence only; no Rhino scans or QA sweep.
        rhino = types.SimpleNamespace(UnitSystem=types.SimpleNamespace(Millimeters="mm"),
                                     RhinoApp=types.SimpleNamespace(Version="test double"))
        sc = types.SimpleNamespace(doc=types.SimpleNamespace(ModelUnitSystem="mm", ModelAbsoluteTolerance=0.01,
                                                             Views=types.SimpleNamespace(Redraw=lambda: None)))
        with mock.patch.dict(sys.modules, {"Rhino": rhino, "rhinoscriptsyntax": types.ModuleType("rs"), "scriptcontext": sc}):
            module = load_rhino_script("06_generate_y_validation_batch.py")
        observed = []
        nominal_ids = ["nominal_web", "nominal_flange"]
        def build(shift, ids, zoom):
            self.assertEqual(ids, nominal_ids)
            observed.append(shift)
            return ["new_web", "new_flange"]
        def sample(ids, path, seed, display, overwrite):
            self.assertEqual(ids, ["new_web", "new_flange"])
            self.assertEqual(seed, 42)
            self.assertFalse(display)
            self.assertFalse(overwrite)
            Path(path).write_text("mock scan, not measurement data")
            return {"point_count": 30000}
        def export(path):
            Path(path).write_text("mock nominal reference")
        modules = {"02": {"build_asbuilt": build},
                   "03": {"generate_synthetic_scan": sample, "POINT_COUNT": 30000, "NOISE_SIGMA_MM": 0.35, "RANDOM_SEED": 42},
                   "04": {"export_nominal_mesh": export}, "05": {"export_component_manifest": export}}
        with tempfile.TemporaryDirectory() as folder:
            with mock.patch.object(module, "PROJECT_ROOT", folder), \
                 mock.patch.object(module, "collect_nominal_parts", return_value=(nominal_ids, [])), \
                 mock.patch.object(module, "verify_asbuilt") as verify, \
                 mock.patch.object(module.runpy, "run_path", side_effect=lambda path, **kw: modules[Path(path).name[:2]]), \
                 contextlib.redirect_stdout(io.StringIO()):
                run_dir = Path(module.generate_batch())
            self.assertEqual(observed, [-16, -12, -8, -4, 0, 4, 8, 12, 16])
            self.assertEqual(verify.call_count, 9)
            metadata = validation.read_json(run_dir / "run_metadata.json")
            self.assertEqual(metadata["generation_status"], "complete")
            self.assertEqual(len(metadata["completed_case_ids"]), 9)
            for i, case_id in enumerate(metadata["case_ids"]):
                case = run_dir / "cases" / case_id
                ground_truth = validation.read_json(case / "ground_truth" / "ground_truth.json")
                self.assertEqual(ground_truth["injected_shift_y_mm"], observed[i])
                self.assertEqual(ground_truth["scan_sha256"], validation.file_hash(case / "input" / "scan.xyz"))
                self.assertFalse((case / "prediction").exists())

    def test_ironpython_compatible_grammar(self):
        from lib2to3.refactor import RefactoringTool
        parser = RefactoringTool([])
        for path in (ROOT / "rhino_scripts").glob("*.py"):
            source = path.read_text(encoding="utf-8")
            parser.refactor_string(source + "\n", str(path))
            self.assertFalse(any(isinstance(n, (ast.JoinedStr, ast.AnnAssign, ast.AsyncFunctionDef))
                                 for n in ast.walk(ast.parse(source))))

    def test_sampling_expressions_and_random_draw_order_unchanged(self):
        old = ast.parse((REGRESSION / "provenance" / "03_before.py.txt").read_text())
        new = ast.parse((ROOT / "rhino_scripts" / "03_generate_synthetic_scan.py").read_text())
        def sampling_loop(tree):
            return next(n for n in ast.walk(tree) if isinstance(n, ast.For) and isinstance(n.iter, ast.Call)
                        and isinstance(n.iter.func, ast.Name) and n.iter.func.id == "range"
                        and any(isinstance(a, ast.Name) and a.id == "POINT_COUNT" for a in n.iter.args))
        self.assertEqual(ast.dump(sampling_loop(old)), ast.dump(sampling_loop(new)))
        for tree in (old, new):
            settings = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
                        if isinstance(n, ast.Assign) and n.targets[0].id in ("POINT_COUNT", "NOISE_SIGMA_MM", "RANDOM_SEED")}
            self.assertEqual(settings, {"POINT_COUNT": 30000, "NOISE_SIGMA_MM": 0.35, "RANDOM_SEED": 42})

    def test_builder_always_copies_nominal_and_deletes_previous_case(self):
        rs = types.ModuleType("rhinoscriptsyntax")
        nominal = {"web": 1200.0, "flange": 1200.0}
        copies = {}
        rs.IsLayer = lambda layer: True
        rs.ObjectsByLayer = lambda layer: list(copies) if layer == "05_ASBUILT_GEOMETRY" else []
        rs.DeleteObjects = lambda ids: copies.clear()
        rs.GetUserText = lambda obj, key: "STF_03" if key == "ASSEMBLY_ID" else "STF_03_" + obj.upper()
        rs.ObjectName = lambda *args: args[0]
        def copy(obj):
            self.assertIn(obj, nominal)
            copies[obj + "_copy"] = nominal[obj]
            return obj + "_copy"
        rs.CopyObject = copy
        rs.ObjectLayer = lambda *args: None
        rs.SetUserText = lambda *args: None
        def move(obj, vector):
            copies[obj] += vector[1]
            return obj
        rs.MoveObject = move
        with mock.patch.dict(sys.modules, {"rhinoscriptsyntax": rs}):
            module = load_rhino_script("02_build_asbuilt_test.py")
            with contextlib.redirect_stdout(io.StringIO()):
                module.build_asbuilt(-16, list(nominal), zoom=False)
                self.assertEqual(list(copies.values()), [1184, 1184])
                module.build_asbuilt(16, list(nominal), zoom=False)
                self.assertEqual(list(copies.values()), [1216, 1216])
                rs.MoveObject = lambda *args: self.fail("Zero control must not apply a transform")
                module.build_asbuilt(0, list(nominal), zoom=False)
                self.assertEqual(list(copies.values()), [1200, 1200])
                rs.DeleteObjects = lambda ids: None
                with self.assertRaisesRegex(RuntimeError, "previous AS-BUILT"):
                    module.build_asbuilt(-8, list(nominal), zoom=False)


if __name__ == "__main__":
    unittest.main()
