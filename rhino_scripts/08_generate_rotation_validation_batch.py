# -*- coding: utf-8 -*-
"""Generate independent local STF_03 rotation cases in Rhino 7."""

import datetime
import hashlib
import json
import math
import os
import runpy
import shutil
import uuid

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


TARGET_ASSEMBLY = "STF_03"
ROTATION_CENTER_MM = (2000.0, 1200.0, 92.0)
ROTATIONS_DEG = (
    (0.0, 0.0, 0.0),
    (-5.0, 0.0, 0.0), (-1.5, 0.0, 0.0),
    (1.5, 0.0, 0.0), (5.0, 0.0, 0.0),
    (0.0, -0.75, 0.0), (0.0, -0.25, 0.0),
    (0.0, 0.25, 0.0), (0.0, 0.75, 0.0),
    (0.0, 0.0, -0.75), (0.0, 0.0, -0.25),
    (0.0, 0.0, 0.25), (0.0, 0.0, 0.75),
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


def euler_xyz_matrix(rotation_deg):
    rx, ry, rz = [math.radians(value) for value in rotation_deg]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    mx = ((1.0, 0.0, 0.0), (0.0, cx, -sx), (0.0, sx, cx))
    my = ((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy))
    mz = ((cz, -sz, 0.0), (sz, cz, 0.0), (0.0, 0.0, 1.0))

    def multiply(a, b):
        return [[sum(a[row][k] * b[k][column] for k in range(3))
                 for column in range(3)] for row in range(3)]
    return multiply(mz, multiply(my, mx))


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
        if not part or part in seen:
            raise RuntimeError("Missing or duplicate PART_ID: {}".format(part))
        seen.add(part)
        parts.append({"part_id": part,
                      "assembly_id": rs.GetUserText(obj, "ASSEMBLY_ID") or None,
                      "bbox_mm": bbox_values(obj)})
    if len(parts) != 11:
        raise RuntimeError("Expected one plate and ten stiffener parts.")
    return objects, parts


def verify_case(created, nominal_parts, rotation_deg):
    if len(created) != len(nominal_parts):
        raise RuntimeError("Incomplete AS-BUILT copy.")
    tolerance = sc.doc.ModelAbsoluteTolerance
    target_changed = False
    for obj, nominal in zip(created, nominal_parts):
        if rs.GetUserText(obj, "PART_ID") != nominal["part_id"]:
            raise RuntimeError("AS-BUILT part identity changed.")
        actual = bbox_values(obj)
        if nominal["assembly_id"] != TARGET_ASSEMBLY:
            for edge in range(2):
                for axis in range(3):
                    if abs(actual[edge][axis] - nominal["bbox_mm"][edge][axis]) > tolerance:
                        raise RuntimeError("A non-target part moved during rotation generation.")
        elif rotation_deg != (0.0, 0.0, 0.0):
            target_changed = target_changed or any(
                abs(actual[edge][axis] - nominal["bbox_mm"][edge][axis]) > tolerance
                for edge in range(2) for axis in range(3))
    if rotation_deg != (0.0, 0.0, 0.0) and not target_changed:
        raise RuntimeError("Target bounds did not change after requested rotation.")


def generate_batch():
    if sc.doc.ModelUnitSystem != Rhino.UnitSystem.Millimeters:
        raise RuntimeError("Rhino document must use millimetres.")
    nominal_ids, nominal_parts = collect_nominal_parts()
    modules = {}
    for number, name in ((2, "02_build_asbuilt_test.py"),
                         (3, "03_generate_synthetic_scan.py"),
                         (4, "04_export_nominal_mesh.py"),
                         (5, "05_export_component_manifest.py")):
        modules[number] = runpy.run_path(
            os.path.join(SCRIPT_DIR, name), run_name="shipqa_rotation_{}".format(number))
    sampler = modules[3]
    if (sampler["POINT_COUNT"], sampler["NOISE_SIGMA_MM"],
            sampler["RANDOM_SEED"]) != (30000, 0.35, 42):
        raise RuntimeError("Rotation sweep requires 30000 points, sigma 0.35 mm and seed 42.")

    run_id = "component_rotation_{}_{}".format(
        datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"), uuid.uuid4().hex[:8])
    run_dir = os.path.join(PROJECT_ROOT, "data", "validation", run_id)
    os.makedirs(run_dir)
    for folder in ("reference", "cases", "provenance"):
        os.mkdir(os.path.join(run_dir, folder))
    case_ids = ["case_{:03d}".format(i + 1) for i in range(len(ROTATIONS_DEG))]
    metadata = {
        "schema_version": 1, "experiment": "independent_component_rotation",
        "run_id": run_id, "generation_status": "incomplete", "units": "mm",
        "angle_units": "degrees", "point_count": 30000,
        "noise_sigma_mm": 0.35, "seed": 42,
        "case_ids": case_ids, "completed_case_ids": [],
        "rotation_center_mm": list(ROTATION_CENTER_MM),
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
            "component_manifest.json": file_hash(manifest)}
        write_json(metadata_path, metadata)
        for case_id, rotation in zip(case_ids, ROTATIONS_DEG):
            case_dir = os.path.join(run_dir, "cases", case_id)
            os.mkdir(case_dir)
            os.mkdir(os.path.join(case_dir, "input"))
            os.mkdir(os.path.join(case_dir, "ground_truth"))
            print("Generating {}: rotation {} deg".format(case_id, rotation))
            created = modules[2]["build_asbuilt"](
                0.0, nominal_ids, zoom=False, shift_xyz_mm=(0.0, 0.0, 0.0),
                target_assembly=TARGET_ASSEMBLY, rotation_xyz_deg=rotation,
                rotation_center_mm=ROTATION_CENTER_MM)
            verify_case(created, nominal_parts, rotation)
            scan_path = os.path.join(case_dir, "input", "scan.xyz")
            generation = sampler["generate_synthetic_scan"](
                created, scan_path, seed=42, display=False, overwrite=False)
            category = "zero_defect_control" if rotation == (0.0, 0.0, 0.0) else (
                "sub_threshold_characterization" if rotation[0] != 0.0
                and abs(rotation[0]) <= 1.5 else "detectable_rotation")
            truth = {
                "schema_version": 1, "units": "mm", "angle_units": "degrees",
                "category": category, "injected_component": TARGET_ASSEMBLY,
                "injected_rotation_xyz_deg": list(rotation),
                "rotation_matrix_nominal_to_observed": euler_xyz_matrix(rotation),
                "rotation_center_mm": list(ROTATION_CENTER_MM),
                "scan_sha256": file_hash(scan_path), "generation": generation}
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
    print("Generated all rotation scans. External QA validation has NOT been run.")
    return run_dir


if __name__ == "__main__":
    generate_batch()
