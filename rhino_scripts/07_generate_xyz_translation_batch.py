# -*- coding: utf-8 -*-
"""Generate controlled STF_03 XYZ translations in Rhino 7, sequentially."""

import datetime
import hashlib
import json
import os
import runpy
import shutil
import uuid

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


TARGET_ASSEMBLY = "STF_03"
VECTORS_MM = (
    (0.0, 0.0, 0.0),
    (-16.0, 0.0, 0.0), (-8.0, 0.0, 0.0),
    (8.0, 0.0, 0.0), (16.0, 0.0, 0.0),
    (0.0, -16.0, 0.0), (0.0, -8.0, 0.0),
    (0.0, 8.0, 0.0), (0.0, 16.0, 0.0),
    (0.0, 0.0, -16.0), (0.0, 0.0, -8.0),
    (0.0, 0.0, 8.0), (0.0, 0.0, 16.0),
)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)


def file_hash(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def write_json(path, value):
    with open(path, "w") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def bbox_values(obj):
    corners = rs.BoundingBox(obj)
    if not corners:
        raise RuntimeError("Cannot read object bounds.")
    return [[min(getattr(p, axis) for p in corners) for axis in ("X", "Y", "Z")],
            [max(getattr(p, axis) for p in corners) for axis in ("X", "Y", "Z")]]


def collect_nominal_parts():
    objects = []
    for layer in ("00_NOMINAL_PLATE", "01_NOMINAL_STIFFENERS"):
        objects.extend(rs.ObjectsByLayer(layer) or [])
    parts = []
    seen = set()
    for obj in objects:
        part = rs.GetUserText(obj, "PART_ID")
        assembly = rs.GetUserText(obj, "ASSEMBLY_ID") or None
        if not part or part in seen:
            raise RuntimeError("Missing or duplicate nominal PART_ID: {}".format(part))
        seen.add(part)
        parts.append({"part_id": part, "assembly_id": assembly,
                      "bbox_mm": bbox_values(obj)})
    if len(parts) != 11:
        raise RuntimeError("Expected one plate and ten stiffener parts.")
    return objects, parts


def verify_asbuilt(created, nominal_parts, vector):
    if len(created) != len(nominal_parts):
        raise RuntimeError("Incomplete AS-BUILT copy.")
    tolerance = sc.doc.ModelAbsoluteTolerance
    for obj, nominal in zip(created, nominal_parts):
        if rs.GetUserText(obj, "PART_ID") != nominal["part_id"]:
            raise RuntimeError("AS-BUILT part identity changed.")
        actual = bbox_values(obj)
        applied = vector if nominal["assembly_id"] == TARGET_ASSEMBLY else (0.0, 0.0, 0.0)
        for edge in range(2):
            for axis in range(3):
                expected = nominal["bbox_mm"][edge][axis] + applied[axis]
                if abs(actual[edge][axis] - expected) > tolerance:
                    raise RuntimeError("AS-BUILT bounds do not match requested XYZ translation.")


def generate_batch():
    if sc.doc.ModelUnitSystem != Rhino.UnitSystem.Millimeters:
        raise RuntimeError("Rhino document must use millimetres; no conversion is performed.")
    nominal_ids, nominal_parts = collect_nominal_parts()
    modules = {}
    for number, name in ((2, "02_build_asbuilt_test.py"),
                         (3, "03_generate_synthetic_scan.py"),
                         (4, "04_export_nominal_mesh.py"),
                         (5, "05_export_component_manifest.py")):
        modules[number] = runpy.run_path(
            os.path.join(SCRIPT_DIR, name), run_name="shipqa_xyz_{}".format(number))
    sampler = modules[3]
    if (sampler["POINT_COUNT"], sampler["NOISE_SIGMA_MM"],
            sampler["RANDOM_SEED"]) != (30000, 0.35, 42):
        raise RuntimeError("XYZ sweep requires 30000 points, sigma 0.35 mm and seed 42.")

    run_id = "xyz_translation_{}_{}".format(
        datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"), uuid.uuid4().hex[:8])
    run_dir = os.path.join(PROJECT_ROOT, "data", "validation", run_id)
    os.makedirs(run_dir)
    for folder in ("reference", "cases", "provenance"):
        os.mkdir(os.path.join(run_dir, folder))
    case_ids = ["case_{:03d}".format(i + 1) for i in range(len(VECTORS_MM))]
    metadata = {
        "schema_version": 1, "experiment": "independent_xyz_component_translation",
        "run_id": run_id, "generation_status": "incomplete", "units": "mm",
        "point_count": 30000, "noise_sigma_mm": 0.35, "seed": 42,
        "target_is_not_an_engine_input": True,
        "case_ids": case_ids, "completed_case_ids": [],
        "nominal_parts_in_sampling_order": nominal_parts,
        "cad_verification_tolerance_mm": sc.doc.ModelAbsoluteTolerance,
        "rhino_version": str(Rhino.RhinoApp.Version), "source_sha256": {},
    }
    metadata_path = os.path.join(run_dir, "run_metadata.json")
    write_json(metadata_path, metadata)
    try:
        for name in sorted(os.listdir(SCRIPT_DIR)):
            if name.endswith(".py"):
                source = os.path.join(SCRIPT_DIR, name)
                shutil.copyfile(source, os.path.join(run_dir, "provenance", name + ".txt"))
                metadata["source_sha256"][name] = file_hash(source)
        mesh = os.path.join(run_dir, "reference", "nominal_reference.obj")
        manifest = os.path.join(run_dir, "reference", "component_manifest.json")
        modules[4]["export_nominal_mesh"](mesh)
        modules[5]["export_component_manifest"](manifest)
        metadata["reference_sha256"] = {
            "nominal_reference.obj": file_hash(mesh),
            "component_manifest.json": file_hash(manifest),
        }
        write_json(metadata_path, metadata)
        for case_id, vector in zip(case_ids, VECTORS_MM):
            case_dir = os.path.join(run_dir, "cases", case_id)
            os.mkdir(case_dir)
            os.mkdir(os.path.join(case_dir, "input"))
            os.mkdir(os.path.join(case_dir, "ground_truth"))
            print("Generating {}: {} mm".format(case_id, vector))
            created = modules[2]["build_asbuilt"](
                0.0, nominal_ids, zoom=False, shift_xyz_mm=vector,
                target_assembly=TARGET_ASSEMBLY)
            verify_asbuilt(created, nominal_parts, vector)
            scan_path = os.path.join(case_dir, "input", "scan.xyz")
            generation = sampler["generate_synthetic_scan"](
                created, scan_path, seed=42, display=False, overwrite=False)
            if generation["point_count"] != 30000:
                raise RuntimeError("Incomplete scan export.")
            truth = {
                "schema_version": 1, "units": "mm",
                "category": "zero_defect_control" if vector == (0.0, 0.0, 0.0)
                else "detectable_translation",
                "injected_component": TARGET_ASSEMBLY,
                "injected_translation_xyz_mm": list(vector),
                "scan_sha256": file_hash(scan_path), "generation": generation,
            }
            write_json(os.path.join(case_dir, "ground_truth", "ground_truth.json"), truth)
            metadata["completed_case_ids"].append(case_id)
            write_json(metadata_path, metadata)
        metadata["generation_status"] = "complete"
    except Exception as error:
        metadata["generation_status"] = "failed"
        metadata["generation_error"] = str(error)
        raise
    finally:
        write_json(metadata_path, metadata)
        sc.doc.Views.Redraw()
        print("Validation run directory: {}".format(run_dir))
    print("Generated all XYZ scans. External QA validation has NOT been run.")
    return run_dir


if __name__ == "__main__":
    generate_batch()
