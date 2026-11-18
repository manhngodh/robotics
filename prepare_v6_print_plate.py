#!/usr/bin/env python3
"""
Orient and arrange all 4 parts of arm_base_v6 ("join base + base")
onto a single 260 x 260 mm Creality SPARKX i7 build plate.
Exports individual print-ready STLs and a combined PrintPlate_V6_BaseAndJoint.stl.
"""
import os
from build123d import *

DIR = "/Users/manhngodh/development/robotics"


def place_on_bed(shape, cx=0.0, cy=0.0):
    bb = shape.bounding_box()
    dx = cx - 0.5 * (bb.min.X + bb.max.X)
    dy = cy - 0.5 * (bb.min.Y + bb.max.Y)
    dz = -bb.min.Z
    return shape.translate((dx, dy, dz))


def main():
    raw_body  = import_stl(os.path.join(DIR, "arm_base_v6_body.stl"))
    raw_joint = import_stl(os.path.join(DIR, "arm_base_v6_joint.stl"))
    raw_jaw   = import_stl(os.path.join(DIR, "arm_base_v6_jaw.stl"))
    raw_screw = import_stl(os.path.join(DIR, "arm_base_v6_screw.stl"))

    # 1. Body (C-clamp): lay on side (+90 deg around X) so continuous XY perimeters wrap C-spine (max strength!)
    p_body = place_on_bed(raw_body.rotate(Axis.X, 90), cx=-55.0, cy=-40.0)

    # 2. Joint (standing servo pylon): upright Z so pocket, M3/M4 holes & M3 spigot stay true round
    p_joint = place_on_bed(raw_joint, cx=+48.0, cy=-40.0)

    # 3. Jaw: flipped 180 deg around X so flat face sits on bed and guide rods point straight up (zero support)
    p_jaw = place_on_bed(raw_jaw.rotate(Axis.X, 180), cx=-45.0, cy=+42.0)

    # 4. Screw: upright knob-down so Tr16x3 helical threads print vertically & true round (zero support)
    p_screw = place_on_bed(raw_screw, cx=+45.0, cy=+42.0)

    parts = {
        "PrintReady_V6_Body.stl":  p_body,
        "PrintReady_V6_Joint.stl": p_joint,
        "PrintReady_V6_Jaw.stl":   p_jaw,
        "PrintReady_V6_Screw.stl": p_screw,
    }
    for fname, shp in parts.items():
        path = os.path.join(DIR, fname)
        export_stl(shp, path)
        bb = shp.bounding_box()
        print(f"  {fname:26s}: X=[{bb.min.X:6.1f},{bb.max.X:6.1f}] ({bb.size.X:5.1f}), "
              f"Y=[{bb.min.Y:6.1f},{bb.max.Y:6.1f}] ({bb.size.Y:5.1f}), Z=[{bb.min.Z:4.1f},{bb.max.Z:5.1f}]")

    plate = Compound([p_body, p_joint, p_jaw, p_screw])
    plate_path = os.path.join(DIR, "PrintPlate_V6_BaseAndJoint.stl")
    export_stl(plate, plate_path)
    pbb = plate.bounding_box()
    print(f"\nCombined plate bbox: {pbb.size.X:.1f} x {pbb.size.Y:.1f} x {pbb.size.Z:.1f} mm -> {plate_path}")


if __name__ == "__main__":
    main()
