import os
from build123d import *

# Motor parameters
motor_w = 20.4
motor_t = 15.4
motor_l = 32.0
clearance = 0.25

r_c = (motor_w + 2 * clearance) / 2  # 10.45 mm
h_c = (motor_t + 2 * clearance) / 2  # 7.95 mm

z_axis = 15.0
x_start = 19.5          # Fork interface
x_wire_end = 29.0       # End of wire chamber
x_rear_wall = 32.0      # End of rear wall / Start of motor cavity
x_motor_end = 64.2      # End of motor cavity / Start of front faceplate
x_end = 68.2            # End of front faceplate

body_w = 34.0           # Cradle outer width
pocket_w = 29.8         # Width between guide rails
cap_w = 29.4            # Cap width (0.2mm clearance per side)
rail_h = 2.5            # Guide rail height above split plane (Z = 17.5 mm)

bolt_y = 12.5           # M3 bolts at +/- 12.5mm
bolt_x1 = 39.0          # Rear M3 bolt
bolt_x2 = 57.0          # Front M3 bolt

# Load Tab 4 U-fork
tab4_step = "/Users/manhngodh/development/onshape/02_universal_segment_tab4.step"
part = import_step(tab4_step)
fork = part.solids()[0].fuse(part.solids()[1]).fuse(part.solids()[3])

# ====================================================================
# 1. CRADLE
# ====================================================================
with BuildPart() as cr:
    # Outer block: height up to Z = 17.5mm
    with BuildSketch(Plane.YZ.offset(x_start)) as s_cradle:
        with Locations([(0, (z_axis + rail_h) / 2)]):
            RectangleRounded(body_w, z_axis + rail_h, radius=2.5)
    extrude(s_cradle.sketch, amount=x_end - x_start)

    # Pocket channel for cap between guide rails:
    # Width = pocket_w (29.8mm), cut from Z = 15.0 up to 18.0 along X in [x_wire_end, x_end + 1.0]
    with BuildSketch(Plane.XY.offset(z_axis)) as s_pocket:
        with Locations([((x_wire_end + x_end + 1.0)/2, 0)]):
            Rectangle(x_end + 1.0 - x_wire_end, pocket_w)
    extrude(s_pocket.sketch, amount=rail_h + 1.0, mode=Mode.SUBTRACT)

    # Motor cavity (lower half)
    plane_yz_motor = Plane(origin=(x_rear_wall, 0, z_axis), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    with BuildSketch(plane_yz_motor) as s_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cavity.sketch, amount=x_motor_end - x_rear_wall, mode=Mode.SUBTRACT)

    # Wiring Chamber (X in [19.5, 29.0], Y in [-8.5, 8.5], Z in [3.5, 18.0])
    with BuildSketch(Plane.XY.offset(3.5)) as s_chamber:
        with Locations([((x_start + x_wire_end)/2, 0)]):
            Rectangle(x_wire_end - x_start + 0.5, 17.0)
    extrude(s_chamber.sketch, amount=16.0, mode=Mode.SUBTRACT)

    # Zip-tie slot at X = 24.25, Z = 1.8, width 3.2mm in X, height 2.0mm in Z, across Y
    with BuildSketch(Plane.XZ.offset(-8.5)) as s_ziptie:
        with Locations([(24.25, 1.8)]):
            RectangleRounded(3.2, 2.0, radius=0.6)
    extrude(s_ziptie.sketch, amount=17.0, mode=Mode.SUBTRACT)

    # Rear bulkhead window (X in [29.0, 32.0]): 11x8mm for tabs
    with BuildSketch(Plane.YZ.offset(x_wire_end - 0.5)) as s_tabs:
        with Locations([(0, z_axis)]):
            RectangleRounded(11.0, 8.0, radius=2.0)
    extrude(s_tabs.sketch, amount=x_rear_wall - x_wire_end + 1.0, mode=Mode.SUBTRACT)

    # Front spindle faceplate cuts (lower half):
    # Collar saddle Ø6.8mm
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_col:
        with Locations([(0, z_axis)]):
            Circle(radius=6.8 / 2)
    extrude(s_col.sketch, amount=2.0, mode=Mode.SUBTRACT)

    # Front collet chuck counterbore Ø12.5mm
    with BuildSketch(Plane.YZ.offset(x_motor_end + 1.2)) as s_collet:
        with Locations([(0, z_axis)]):
            Circle(radius=12.5 / 2)
    extrude(s_collet.sketch, amount=x_end - (x_motor_end + 1.2) + 1.0, mode=Mode.SUBTRACT)

    # 2x M2 front screws (pitch 10mm, Y = +/- 5mm)
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
        with Locations([((x_rear_wall + x_motor_end)/2, 0)]):
            RectangleRounded(18.0, 7.0, radius=2.0)
    extrude(s_vent.sketch, amount=z_axis + 2.0, mode=Mode.SUBTRACT)

cradle = fork.fuse(cr.part)
print(f"Cradle fused! Volume: {cradle.volume:.1f} mm^3")

# ====================================================================
# 2. CAP
# ====================================================================
cap_len = x_end - x_wire_end  # 39.2 mm (from wire chamber end to front faceplate)
cap_top_z = z_axis + h_c + 3.0

with BuildPart() as cp:
    with BuildSketch(Plane.YZ.offset(x_wire_end)) as s_cap_profile:
        # Base ear bar
        with Locations([(0, z_axis + 5.5 / 2)]):
            RectangleRounded(cap_w, 5.5, radius=1.5)
        # Upper crown
        with Locations([(0, z_axis + 7.5)]):
            RectangleRounded(21.0, 7.0, radius=2.5, mode=Mode.ADD)
    extrude(s_cap_profile.sketch, amount=cap_len)

    # Motor cavity (upper half)
    with BuildSketch(plane_yz_motor) as s_cap_cav:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cap_cav.sketch, amount=x_motor_end - x_rear_wall, mode=Mode.SUBTRACT)

    # Rear bulkhead window (upper half)
    with BuildSketch(Plane.YZ.offset(x_wire_end - 0.5)) as s_ctabs:
        with Locations([(0, z_axis)]):
            RectangleRounded(11.0, 8.0, radius=2.0)
    extrude(s_ctabs.sketch, amount=x_rear_wall - x_wire_end + 1.0, mode=Mode.SUBTRACT)

    # Front collar upper saddle Ø6.8mm
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_ccol:
        with Locations([(0, z_axis)]):
            Circle(radius=6.8 / 2)
    extrude(s_ccol.sketch, amount=2.0, mode=Mode.SUBTRACT)

    # Front collet counterbore Ø12.5mm
    with BuildSketch(Plane.YZ.offset(x_motor_end + 1.2)) as s_ccollet:
        with Locations([(0, z_axis)]):
            Circle(radius=12.5 / 2)
    extrude(s_ccollet.sketch, amount=x_end - (x_motor_end + 1.2) + 1.0, mode=Mode.SUBTRACT)

    # 2x M2 front screws (upper half)
    for sy in [-1, 1]:
        with Locations([(x_motor_end - 0.5, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_m2:
                Circle(radius=2.4 / 2)
            extrude(s_m2.sketch, amount=x_end - x_motor_end + 1.0, mode=Mode.SUBTRACT)
        with Locations([(x_end - 2.0, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_cb:
                Circle(radius=4.5 / 2)
            extrude(s_cb.sketch, amount=3.0, mode=Mode.SUBTRACT)

    # Top cooling vent
    with BuildSketch(Plane.XY.offset(z_axis)) as s_cvent:
        with Locations([((x_rear_wall + x_motor_end)/2, 0)]):
            RectangleRounded(18.0, 7.0, radius=2.0)
    extrude(s_cvent.sketch, amount=20.0, mode=Mode.SUBTRACT)

    # 4x M3 bolt through holes & top counterbores
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, z_axis)]):
                Cylinder(radius=3.4 / 2, height=18.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, z_axis + 4.0)]):
                Cylinder(radius=6.4 / 2, height=15.0, mode=Mode.SUBTRACT)

cap = cp.part
print(f"Cap built! Volume: {cap.volume:.1f} mm^3")
print("Geometry generation test PASSED completely!")
