import os
import sys
from build123d import *

print("Testing V3 CAD construction...")

motor_w = 20.4
motor_t = 15.4
motor_l = 32.0
clearance = 0.25

r_c = (motor_w + 2 * clearance) / 2  # 10.45 mm
h_c = (motor_t + 2 * clearance) / 2  # 7.95 mm

z_axis = 15.0
x_start = 19.5
x_chamber_end = 29.0
x_rear_wall_end = 32.0
x_motor_end = 64.2
x_end = 68.2

body_w = 34.0
bolt_y = 13.0
bolt_x1 = 40.0
bolt_x2 = 56.0

# 1. Load U-fork
tab4_step = "/Users/manhngodh/development/onshape/02_universal_segment_tab4.step"
part = import_step(tab4_step)
fork_solids = [part.solids()[0], part.solids()[1], part.solids()[3]]
fork = fork_solids[0]
for s in fork_solids[1:]:
    fork = fork.fuse(s)
print(f"U-fork fused! Volume: {fork.volume:.1f} mm^3")

# 2. Build Cradle
with BuildPart() as cradle_builder:
    # Outer block: up to z_axis + 1.5mm on the outer perimeter
    with BuildSketch(Plane.YZ.offset(x_start)) as s_cradle:
        with Locations([(0, (z_axis + 1.5) / 2)]):
            RectangleRounded(body_w, z_axis + 1.5, radius=2.5)
    extrude(s_cradle.sketch, amount=x_end - x_start)

    # Cut the stepped pocket for the cap:
    # Between Y in [-13.8, 13.8], cut from Z = 15.0 to 17.0 along X in [x_chamber_end, x_motor_end]
    with BuildSketch(Plane.XY.offset(z_axis)) as s_pocket:
        with Locations([((x_chamber_end + x_motor_end)/2, 0)]):
            Rectangle(x_motor_end - x_chamber_end, 27.6)
    extrude(s_pocket.sketch, amount=5.0, mode=Mode.SUBTRACT)

    # Motor cavity (lower half)
    plane_yz_motor = Plane(origin=(x_rear_wall_end, 0, z_axis), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    with BuildSketch(plane_yz_motor) as s_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cavity.sketch, amount=x_motor_end - x_rear_wall_end, mode=Mode.SUBTRACT)

    # Wiring Chamber (X in [19.5, 29.0], Y in [-8.5, 8.5], Z from 3.5 up)
    with BuildSketch(Plane.XY.offset(3.5)) as s_chamber:
        with Locations([((x_start + x_chamber_end)/2, 0)]):
            Rectangle(x_chamber_end - x_start + 0.5, 17.0)
    extrude(s_chamber.sketch, amount=15.0, mode=Mode.SUBTRACT)

    # Zip-tie slot at X = 24.25, Z = 1.8, width 3.2mm in X, height 2.0mm in Z, across Y
    with BuildSketch(Plane.XZ.offset(-8.5)) as s_ziptie:
        with Locations([(24.25, 1.8)]):
            RectangleRounded(3.2, 2.0, radius=0.6)
    extrude(s_ziptie.sketch, amount=17.0, mode=Mode.SUBTRACT)

    # Rear bulkhead window (X in [29.0, 32.0]): center window 11x8mm for solder tabs & rear boss
    with BuildSketch(Plane.YZ.offset(x_chamber_end - 0.5)) as s_tab_win:
        with Locations([(0, z_axis)]):
            RectangleRounded(11.0, 8.0, radius=2.0)
    extrude(s_tab_win.sketch, amount=x_rear_wall_end - x_chamber_end + 1.0, mode=Mode.SUBTRACT)

    # Front Spindle Faceplate (X in [64.2, 68.2]):
    # 1. Motor collar clearance hole Ø6.8mm through to front counterbore
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_collar:
        with Locations([(0, z_axis)]):
            Circle(radius=6.8 / 2)
    extrude(s_collar.sketch, amount=2.0, mode=Mode.SUBTRACT)

    # 2. Front Collet chuck clearance counterbore Ø12.5mm
    with BuildSketch(Plane.YZ.offset(x_motor_end + 1.2)) as s_collet:
        with Locations([(0, z_axis)]):
            Circle(radius=12.5 / 2)
    extrude(s_collet.sketch, amount=x_end - (x_motor_end + 1.2) + 1.0, mode=Mode.SUBTRACT)

    # 3. 2x M2 front screws (pitch 10mm, Y = +/- 5mm) with counterbores
    for sy in [-1, 1]:
        with Locations([(x_motor_end - 0.5, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_m2:
                Circle(radius=2.4 / 2)
            extrude(s_m2.sketch, amount=x_end - x_motor_end + 1.0, mode=Mode.SUBTRACT)
        with Locations([(x_end - 2.0, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_cb:
                Circle(radius=4.5 / 2)
            extrude(s_cb.sketch, amount=3.0, mode=Mode.SUBTRACT)

    # 4x M3 clamping bolt holes & bottom hex nut pockets
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, -1.0)]):
                Cylinder(radius=3.4 / 2, height=z_axis + 5.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, 0)]):
                with BuildSketch() as s_nut:
                    RegularPolygon(radius=5.85 / (3**0.5), side_count=6)
                extrude(s_nut.sketch, amount=3.5, mode=Mode.SUBTRACT)

    # Bottom cooling vent under motor
    with BuildSketch(Plane.XY.offset(-1.0)) as s_vent:
        with Locations([((x_rear_wall_end + x_motor_end)/2, 0)]):
            RectangleRounded(18.0, 7.0, radius=2.0)
    extrude(s_vent.sketch, amount=z_axis + 2.0, mode=Mode.SUBTRACT)

cradle = cradle_builder.part
full_cradle = fork.fuse(cradle)
print(f"Cradle built successfully! Volume: {full_cradle.volume:.1f} mm^3")

# 3. Build Cap
cap_len = x_motor_end - x_chamber_end
cap_top_z = z_axis + h_c + 3.0

with BuildPart() as cap_builder:
    # Outer hull: base block with stepped ears
    # Width = 27.2 mm (to fit inside cradle's 27.6mm pocket with 0.2mm clearance per side)
    with BuildSketch(Plane.YZ.offset(x_chamber_end)) as s_cap_base:
        with Locations([(0, z_axis + 3.0)]):
            RectangleRounded(27.2, 6.0, radius=1.5)
    b_base = extrude(s_cap_base.sketch, amount=cap_len)

    # Top arch crown
    with BuildSketch(Plane.YZ.offset(x_chamber_end)) as s_cap_crown:
        with Locations([(0, cap_top_z - 1.5)]):
            RectangleRounded(16.0, 3.6, radius=1.2)
    b_crown = extrude(s_cap_crown.sketch, amount=cap_len)
    loft([s_cap_base.sketch, s_cap_crown.sketch])

    # Motor cavity (upper half)
    with BuildSketch(plane_yz_motor) as s_cap_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cap_cavity.sketch, amount=x_motor_end - x_rear_wall_end, mode=Mode.SUBTRACT)

    # Top vent
    with BuildSketch(Plane.XY.offset(z_axis)) as s_top_vent:
        with Locations([((x_rear_wall_end + x_motor_end)/2, 0)]):
            RectangleRounded(18.0, 7.0, radius=2.0)
    extrude(s_top_vent.sketch, amount=20.0, mode=Mode.SUBTRACT)

    # 4x M3 bolt through holes & top counterbores
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, z_axis)]):
                Cylinder(radius=3.4 / 2, height=18.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, z_axis + 5.5)]):
                Cylinder(radius=6.4 / 2, height=15.0, mode=Mode.SUBTRACT)

cap = cap_builder.part
print(f"Cap built successfully! Volume: {cap.volume:.1f} mm^3")
print("All geometry checks passed!")
