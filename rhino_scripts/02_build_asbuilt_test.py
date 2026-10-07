# -*- coding: utf-8 -*-

import rhinoscriptsyntax as rs


ASBUILT_LAYER = "05_ASBUILT_GEOMETRY"
KNOWN_SHIFT_MM = -8.0


def ensure_layer(name):
    if not rs.IsLayer(name):
        rs.AddLayer(name)


def build_asbuilt(shift_mm=KNOWN_SHIFT_MM, nominal_objects=None, zoom=True,
                  shift_xyz_mm=None, target_assembly="STF_03",
                  rotation_xyz_deg=None, rotation_center_mm=None):
    if shift_xyz_mm is None:
        shift_xyz_mm = (0.0, float(shift_mm), 0.0)
    if len(shift_xyz_mm) != 3:
        raise ValueError("shift_xyz_mm must contain X, Y and Z in millimetres.")
    shift_xyz_mm = tuple(float(value) for value in shift_xyz_mm)
    if rotation_xyz_deg is None:
        rotation_xyz_deg = (0.0, 0.0, 0.0)
    if len(rotation_xyz_deg) != 3:
        raise ValueError("rotation_xyz_deg must contain RX, RY and RZ in degrees.")
    rotation_xyz_deg = tuple(float(value) for value in rotation_xyz_deg)
    if any(value != 0.0 for value in rotation_xyz_deg) and rotation_center_mm is None:
        raise ValueError("rotation_center_mm is required for a controlled rotation.")
    ensure_layer(ASBUILT_LAYER)

    # Remove previous AS-BUILT test geometry
    old_asbuilt_objects = rs.ObjectsByLayer(ASBUILT_LAYER)

    if old_asbuilt_objects:
        rs.DeleteObjects(old_asbuilt_objects)
        if rs.ObjectsByLayer(ASBUILT_LAYER):
            raise RuntimeError("Could not remove all previous AS-BUILT geometry.")
    
    
    if nominal_objects is None:
        nominal_objects = []
        for layer_name in ["00_NOMINAL_PLATE", "01_NOMINAL_STIFFENERS"]:
            nominal_objects.extend(rs.ObjectsByLayer(layer_name) or [])


    if not nominal_objects:
        raise RuntimeError("Nominal geometry not found.")

    else:

        created = []

        for obj in nominal_objects:

            part_id = rs.GetUserText(obj, "PART_ID")
            assembly_id = rs.GetUserText(obj, "ASSEMBLY_ID")
            original_name = rs.ObjectName(obj)

            new_obj = rs.CopyObject(obj)

            if not new_obj:
                raise RuntimeError("Could not copy nominal part: {}".format(part_id))

            rs.ObjectLayer(new_obj, ASBUILT_LAYER)

            # As-built metadata
            if part_id:
                rs.SetUserText(new_obj, "PART_ID", part_id)

            if assembly_id:
                rs.SetUserText(new_obj, "ASSEMBLY_ID", assembly_id)

            rs.SetUserText(new_obj, "MODEL_TYPE", "AS_BUILT")

            if original_name:
                rs.ObjectName(new_obj, original_name + " [AS-BUILT]")

            # ------------------------------------------------
            # CONTROLLED DEFECT
            # Translate the complete STF_03 assembly by shift_mm in Y.
            # ------------------------------------------------

            if assembly_id == target_assembly:

                if rotation_center_mm is not None:
                    center = tuple(float(value) for value in rotation_center_mm)
                    axes = ((1.0, 0.0, 0.0),
                            (0.0, 1.0, 0.0),
                            (0.0, 0.0, 1.0))
                    for angle, axis in zip(rotation_xyz_deg, axes):
                        if angle != 0.0:
                            rotated = rs.RotateObject(new_obj, center, angle, axis, copy=False)
                            if not rotated:
                                raise RuntimeError("Could not rotate part: {}".format(part_id))

                # The zero-defect control is an unmodified nominal copy.
                if any(value != 0.0 for value in shift_xyz_mm):
                    moved = rs.MoveObject(
                        new_obj,
                        shift_xyz_mm
                    )
                    if not moved:
                        raise RuntimeError("Could not move part: {}".format(part_id))

                rs.SetUserText(
                    new_obj,
                    "KNOWN_DEFECT",
                    "TRANSVERSE_SHIFT"
                )

                rs.SetUserText(
                    new_obj,
                    "KNOWN_SHIFT_Y_MM",
                    str(shift_xyz_mm[1])
                )

                rs.SetUserText(new_obj, "KNOWN_SHIFT_X_MM", str(shift_xyz_mm[0]))
                rs.SetUserText(new_obj, "KNOWN_SHIFT_Z_MM", str(shift_xyz_mm[2]))
                rs.SetUserText(new_obj, "KNOWN_ROTATION_X_DEG", str(rotation_xyz_deg[0]))
                rs.SetUserText(new_obj, "KNOWN_ROTATION_Y_DEG", str(rotation_xyz_deg[1]))
                rs.SetUserText(new_obj, "KNOWN_ROTATION_Z_DEG", str(rotation_xyz_deg[2]))

            created.append(new_obj)


        print("--------------------------------")
        print("AS-BUILT TEST MODEL CREATED")
        print("--------------------------------")
        print("Objects created: {}".format(len(created)))
        print("Controlled defect:")
        print("{} shifted X={:+.3f}, Y={:+.3f}, Z={:+.3f} mm".format(
            target_assembly, shift_xyz_mm[0], shift_xyz_mm[1], shift_xyz_mm[2]))
        print("{} rotated RX={:+.3f}, RY={:+.3f}, RZ={:+.3f} deg".format(
            target_assembly, rotation_xyz_deg[0], rotation_xyz_deg[1],
            rotation_xyz_deg[2]))
        print("--------------------------------")

        if zoom:
            rs.ZoomExtents()
        return created


if __name__ == "__main__":
    build_asbuilt()
