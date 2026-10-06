# -*- coding: utf-8 -*-

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


NOMINAL_LAYERS = [
    "00_NOMINAL_PLATE",
    "01_NOMINAL_STIFFENERS"
]


# ------------------------------------------------------------
# Collect nominal geometry
# ------------------------------------------------------------

def export_nominal_mesh(save_path=None):
    object_ids = []

    for layer_name in NOMINAL_LAYERS:
        ids = rs.ObjectsByLayer(layer_name)
        if ids:
            object_ids.extend(ids)

    if not object_ids:
        raise RuntimeError("No nominal geometry found.")

    else:

        mesh_params = Rhino.Geometry.MeshingParameters.QualityRenderMesh

        object_meshes = []
        total_vertices = 0
        total_faces = 0
        for obj_id in object_ids:

            rhino_obj = sc.doc.Objects.Find(obj_id)

            if rhino_obj is None:
                raise RuntimeError("Nominal object disappeared during export.")

            geometry = rhino_obj.Geometry

            if isinstance(geometry, Rhino.Geometry.Brep):

                meshes = Rhino.Geometry.Mesh.CreateFromBrep(
                    geometry,
                    mesh_params
                )

                if meshes:
                    meshes = list(meshes)
                else:
                    raise RuntimeError("Could not mesh a nominal object.")

            elif isinstance(geometry, Rhino.Geometry.Mesh):

                meshes = [geometry.DuplicateMesh()]
            else:
                raise RuntimeError("Unsupported nominal geometry type.")


            part_id = rs.GetUserText(obj_id, "PART_ID")
            if not part_id:
                raise RuntimeError("Nominal object has no PART_ID.")
            for mesh in meshes:
                mesh.Faces.ConvertQuadsToTriangles()
                mesh.Compact()
                object_meshes.append((part_id, mesh))
                total_vertices += mesh.Vertices.Count
                total_faces += mesh.Faces.Count


        print("Nominal CAD meshed.")
        print("Vertices: {}".format(total_vertices))
        print("Triangles: {}".format(total_faces))


        # --------------------------------------------------------
        # Select destination
        # --------------------------------------------------------

        if save_path is None:
            save_path = rs.SaveFileName(
                "Save nominal QA reference mesh",
                "Wavefront OBJ (*.obj)|*.obj||"
            )


        if save_path:

            if total_faces == 0:
                raise RuntimeError("Nominal reference has no mesh faces.")
            f = open(save_path, "w")

            f.write("# ShipQA nominal reference mesh\n")
            f.write("# Units: millimetres\n")

            vertex_offset = 0
            for part_id, mesh in object_meshes:
                # Group identity is ignored by the global distance mesh but lets
                # the external engine recover component-specific nominal faces.
                f.write("g {}\n".format(part_id))
                for i in range(mesh.Vertices.Count):
                    v = mesh.Vertices[i]
                    f.write("v {:.6f} {:.6f} {:.6f}\n".format(v.X, v.Y, v.Z))
                for i in range(mesh.Faces.Count):
                    face = mesh.Faces[i]
                    if face.IsTriangle:
                        f.write("f {} {} {}\n".format(
                            face.A + 1 + vertex_offset,
                            face.B + 1 + vertex_offset,
                            face.C + 1 + vertex_offset))
                vertex_offset += mesh.Vertices.Count


            f.close()

            print("--------------------------------")
            print("NOMINAL REFERENCE EXPORTED")
            print("--------------------------------")
            print(save_path)
        return save_path


if __name__ == "__main__":
    export_nominal_mesh()
