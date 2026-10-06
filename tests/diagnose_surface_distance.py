"""Reproduce the recorded audit points and isolate the Open3D primitive error.

This diagnostic reads geometry only; it does not run the displacement estimator.
"""

import argparse
from collections import Counter
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import open3d as o3d

from distance_reference import read_obj_triangles, brute_force_obj_distance, box_surface_distances, decimal_triangle_distance


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "data/validation/y_translation_20261006T104604Z_b8c0a39c"
sys.path.insert(0, str(ROOT / "src"))
from surface_distance import compute_surface_deviation


def raycast(vertices, faces, points):
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(np.asarray(vertices, dtype=np.float32)),
                        o3d.core.Tensor(np.asarray(faces, dtype=np.uint32)))
    query = o3d.core.Tensor(np.asarray(points, dtype=np.float32))
    return scene.compute_distance(query).numpy(), scene.compute_closest_points(query)["points"].numpy()


def triangle_multiset(vertices, faces):
    return Counter(tuple(sorted(tuple(v) for v in vertices[face])) for face in faces)


def barycentric_arithmetic(point, triangle, dtype):
    # The interior branch's determinant arithmetic, as in Open3D v0.20.0
    # RaycastingScene.cpp closestPointTriangle. Report the intermediate losses.
    a, b, c = np.asarray(triangle, dtype=dtype)
    p = np.asarray(point, dtype=dtype)
    ab, ac = b - a, c - a
    d1, d2 = ab @ (p-a), ac @ (p-a)
    d3, d4 = ab @ (p-b), ac @ (p-b)
    d5, d6 = ab @ (p-c), ac @ (p-c)
    va, vb, vc = d3*d6-d5*d4, d5*d2-d1*d6, d1*d4-d3*d2
    v, w = vb / (va+vb+vc), vc / (va+vb+vc)
    closest = a + v*ab + w*ac
    return {"va_vb_vc": [float(x) for x in (va, vb, vc)],
            "closest_point_mm": closest.tolist(), "distance_mm": float(np.linalg.norm(p-closest)),
            "vc_large_products": [float(d1*d4), float(d3*d2)]}


def diagnose():
    obj = RUN / "reference/nominal_reference.obj"
    vertices, faces = read_obj_triangles(obj)
    legacy = o3d.io.read_triangle_mesh(str(obj))
    lv, lf = np.asarray(legacy.vertices), np.asarray(legacy.triangles)
    tensor = o3d.t.geometry.TriangleMesh.from_legacy(legacy)
    tv, tf = tensor.vertex.positions.numpy(), tensor.triangle.indices.numpy()
    original = triangle_multiset(vertices, faces)
    assert original == triangle_multiset(lv, lf) == triangle_multiset(tv, tf)
    audit = json.loads((RUN / "distance_audit.json").read_text())
    meta = json.loads((RUN / "run_metadata.json").read_text())
    boxes = [part["bbox_mm"] for part in meta["nominal_parts_in_sampling_order"]]
    points = np.array([case["engine_max_point_xyz_mm"] for case in audit["cases"]])
    # Reproduce the exact original legacy -> tensor -> scene path as well as raw OBJ.
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tensor)
    distances = scene.compute_distance(o3d.core.Tensor(points.astype(np.float32))).numpy()
    selected = scene.compute_closest_points(o3d.core.Tensor(points.astype(np.float32)))["primitive_ids"].numpy()
    corrected = compute_surface_deviation(points, vertices, faces)
    raw_distances, _ = raycast(vertices, faces, points)
    areas2 = np.linalg.norm(np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],
                                    vertices[faces[:,2]]-vertices[faces[:,0]]), axis=1)
    report = {"open3d_version": o3d.__version__, "build": o3d._build_config,
              "obj_vertex_records": len(vertices), "imported_vertices": len(lv),
              "triangle_count": len(faces), "duplicate_geometric_triangles": len(faces)-len(original),
              "zero_area_triangles": int((areas2 == 0).sum()),
              "obj_import_tensor_triangle_coordinates_exactly_equal": True,
              "source": "https://github.com/isl-org/Open3D/blob/v0.20.0/cpp/open3d/t/geometry/RaycastingScene.cpp#L190-L253",
              "recorded_points": []}
    for index, (point, case) in enumerate(zip(points, audit["cases"])):
        exact, closest, triangle_id = brute_force_obj_distance(point, vertices, faces)
        box = float(box_surface_distances(point[None], boxes)[0])
        assert abs(exact-box) < 1e-9
        assert abs(exact-corrected[index]) < 1e-9
        assert abs(float(distances[index])-case["engine_distance_at_point_mm"]) < 1e-6
        report["recorded_points"].append({"case_id": case["case_id"], "point_mm": point.tolist(),
                                         "raycasting_scene_mm": float(distances[index]),
                                         "direct_obj_raycast_mm": float(raw_distances[index]),
                                         "decimal_brute_force_mm": exact, "analytic_box_mm": box,
                                         "corrected_kernel_mm": float(corrected[index]),
                                         "open3d_selected_triangle_id": int(selected[index]),
                                         "decimal_distance_to_open3d_selected_triangle_mm": decimal_triangle_distance(point,lv[lf[selected[index]]])[0],
                                         "decimal_closest_point_mm": closest.tolist(),
                                         "decimal_obj_triangle_id": triangle_id})
    triangle = np.array([[4000.,1240.,172.], [0.,1240.,162.], [0.,1240.,172.]])
    point = np.array([1364.6434,1240.4315,168.5501])
    one_distance, one_closest = raycast(triangle, [[0,1,2]], [point])
    report["minimal_reproduction"] = {"vertices_mm": triangle.tolist(), "point_mm": point.tolist(),
                                       "raycasting_distance_mm": float(one_distance[0]),
                                       "raycasting_closest_point_mm": one_closest[0].tolist(),
                                       "query_float32_rounding_mm": float(np.linalg.norm(point-point.astype(np.float32))),
                                       "float32_arithmetic": barycentric_arithmetic(point,triangle,np.float32),
                                       "float64_arithmetic": barycentric_arithmetic(point,triangle,np.float64),
                                       "permutations": []}
    for permutation in itertools.permutations(range(3)):
        distance, _ = raycast(triangle, [permutation], [point])
        report["minimal_reproduction"]["permutations"].append({"indices": permutation, "distance_mm": float(distance[0])})
    report["minimal_reproduction"]["same_rounded_inputs_decimal_mm"] = brute_force_obj_distance(
        point.astype(np.float32),triangle.astype(np.float32),np.array([[0,1,2]]))[0]
    try:
        scene.compute_distance(o3d.core.Tensor(points, dtype=o3d.core.Dtype.Float64))
        report["float64_query_supported"] = True
    except RuntimeError as error:
        report["float64_query_supported"] = False
        report["float64_rejection"] = str(error)
    return report


def verify_all_scan_points():
    vertices, faces = read_obj_triangles(RUN / "reference/nominal_reference.obj")
    metadata = json.loads((RUN / "run_metadata.json").read_text())
    boxes = [p["bbox_mm"] for p in metadata["nominal_parts_in_sampling_order"]]
    scans = [(cid,RUN / "cases" / cid / "input/scan.xyz") for cid in metadata["case_ids"]]
    scans.append(("preserved_regression",ROOT / "data/regression/stf03_negative_8/input/scan.xyz"))
    checks = []
    for case_id, scan in scans:
        points = np.loadtxt(scan)
        actual = compute_surface_deviation(points,vertices,faces)
        reference = box_surface_distances(points,boxes)
        error = np.abs(actual-reference)
        assert error.max() < 1e-9
        assert np.array_equal(actual>5,reference>5)
        checks.append({"case_id":case_id,"points":len(points),"max_absolute_difference_mm":float(error.max()),
                       "candidate_masks_identical":True})
    return checks


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--verify-all", action="store_true", help="Also check all 300000 archived scan points against the box oracle")
    args = parser.parse_args()
    result = diagnose()
    if args.verify_all:
        result["all_scan_points_verification"] = verify_all_scan_points()
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    for row in result["recorded_points"]:
        print(row["case_id"], "Open3D:", row["raycasting_scene_mm"], "Decimal:", row["decimal_brute_force_mm"])
    print("Saved", args.output)
