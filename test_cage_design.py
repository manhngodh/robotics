import os
from build123d import *

# Motor dimensions
motor_w = 20.4
motor_t = 15.4
motor_l = 32.0
clearance = 0.25

r_c = (motor_w + 2 * clearance) / 2  # 10.45 mm
h_c = (motor_t + 2 * clearance) / 2  # 7.95 mm

z_axis = 15.0
x_start = 19.5          # Fork interface
x_wire_end = 29.0       # End of wire chamber
x_rear_wall = 32.0      # End of rear bulkhead / Start of motor cavity
x_motor_end = 64.2      # End of motor cavity / Start of front faceplate
x_end = 68.2            # End of front faceplate

body_w = 34.0
cap_w = 29.6            # Cap width (fits in cradle pocket Y in [-15.0, 15.0] with 0.2mm clearance)
bolt_y = 12.5           # M3 bolts at +/- 12.5mm
bolt_x1 = 39.0          # Rear M3 bolt
bolt_x2 = 57.0          # Front M3 bolt

# Load Tab 4 U-fork
tab4_step = "/Users/manhngodh/development/onshape/02_universal_segment_tab4.step"
part = import_step(tab4_step)
fork = part.solids()[0].fuse(part.solids()[1]).fuse(part.solids()[3])

# CRADLE
with BuildPart() as cr:
    # 1. Main outer block from x_start to x_end
    # Height goes up to z_axis + 2.5mm (Z=17.5) to form the side guide rails
    with BuildSketch(Plane.YZ.offset(x_start)) as s_cradle:
        with Locations([(0, 17.5 / 2)]):
            RectangleRounded(body_w, 17.5, radius=2.5)
    extrude(s_cradle.sketch, amount=x_end - x_start)

    # 2. Guide rail pocket for the cap:
    # Cut Y in [-14.9, 14.9] from Z = 15.0 up to 18.0 along X in [x_wire_end, x_motor_end]
    with BuildSketch(Plane.XY.offset(z_axis)) as s_pocket:
        with Locations([((x_wire_end + x_motor_end)/2, 0)]):
            Rectangle(x_motor_end - x_wire_end, 29.8)
    extrude(s_pocket.sketch, amount=5.0, mode=Mode.SUBTRACT)

    # 3. Motor cavity (lower half) from x_rear_wall to x_motor_end
    plane_yz_motor = Plane(origin=(x_rear_wall, 0, z_axis), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    with BuildSketch(plane_yz_motor) as s_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cavity.sketch, amount=x_motor_end - x_rear_wall, mode=Mode.SUBTRACT)

    # 4. Wiring Chamber (X in [19.5, 29.0], Y in [-8.5, 8.5], Z in [3.5, 18.0])
    with BuildSketch(Plane.XY.offset(3.5)) as s_chamber:
        with Locations([((x_start + x_wire_end)/2, 0)]):
            Rectangle(x_wire_end - x_start + 0.5, 17.0)
    extrude(s_chamber.sketch, amount=16.0, mode=Mode.SUBTRACT)

    # 5. Zip-tie slot at X = 24.25, Z = 1.8, width 3.2mm in X, height 2.0mm in Z, across Y
    with BuildSketch(Plane.XZ.offset(-8.5)) as s_ziptie:
        with Locations([(24.25, 1.8)]):
            RectangleRounded(3.2, 2.0, radius=0.6)
    extrude(s_ziptie.sketch, amount=17.0, mode=Mode.SUBTRACT)

    # 6. Rear bulkhead center window (X in [29.0, 32.0]): 11x8mm for tabs
    with BuildSketch(Plane.YZ.offset(x_wire_end - 0.5)) as s_tabs:
        with Locations([(0, z_axis)]):
            RectangleRounded(11.0, 8.0, radius=2.0)
    extrude(s_tabs.sketch, amount=x_rear_wall - x_wire_end + 1.0, mode=Mode.SUBTRACT)

    # 7. Front spindle faceplate cuts:
    # 7a. Collar hole Ø6.8mm through wall
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_col:
        with Locations([(0, z_axis)]):
            Circle(radius=6.8 / 2)
    extrude(s_col.sketch, amount=2.0, mode=Mode.SUBTRACT)

    # 7b. Front collet chuck counterbore Ø12.5mm
    with BuildSketch(Plane.YZ.offset(x_motor_end + 1.5)) as s_collet:
        with Locations([(0, z_axis)]):
            Circle(radius=12.5 / 2)
    extrude(s_collet.sketch, amount=x_end - (x_motor_end + 1.5) + 1.0, mode=Mode.SUBTRACT)

    # 7c. 2x M2 front screws (pitch 10mm, Y = +/- 5mm) with counterbores
    for sy in [-1, 1]:
        with Locations([(x_motor_end - 0.5, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_m2:
                Circle(radius=2.4 / 2)
            extrude(s_m2.sketch, amount=x_end - x_motor_end + 1.0, mode=Mode.SUBTRACT)
        with Locations([(x_end - 2.0, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_cb:
                Circle(radius=4.5 / 2)
            extrude(s_cb.sketch, amount=3.0, mode=Mode.SUBTRACT)

    # 8. 4x M3 clamping bolt holes & bottom hex nut pockets
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, -1.0)]):
                Cylinder(radius=3.4 / 2, height=z_axis + 5.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, 0)]):
                with BuildSketch() as s_nut:
                    RegularPolygon(radius=5.85 / (3**0.5), side_count=6)
                extrude(s_nut.sketch, amount=3.5, mode=Mode.SUBTRACT)

    # 9. Bottom cooling vent under motor
    with BuildSketch(Plane.XY.offset(-1.0)) as s_vent:
        with Locations([((x_rear_wall + x_motor_end)/2, 0)]):
            RectangleRounded(18.0, 7.0, radius=2.0)
    extrude(s_vent.sketch, amount=z_axis + 2.0, mode=Mode.SUBTRACT)

cradle_solid = cr.part
full_cradle = fork.fuse(cradle_solid)
print(f"Cradle fused! Volume: {full_cradle.volume:.1f} mm^3")

# CAP
cap_len = x_motor_end - x_wire_end  # 35.2 mm
cap_top_z = z_axis + h_c + 3.0     # 25.95 mm

with BuildPart() as cp:
    # Base block: width 29.4mm (fits inside 29.8mm pocket with 0.2mm clearance per side)
    with BuildSketch(Plane.YZ.offset(x_wire_end)) as s_cbase:
        with Locations([(0, z_axis + 3.0)]):
            RectangleRounded(cap_w, 6.0, radius=1.5)
    b_cbase = extrude(s_cbase.sketch, amount=cap_len)

    # Top arch crown
    with BuildSketch(Plane.YZ.offset(x_wire_end)) as s_ccrown:
        with Locations([(0, cap_top_z - 1.8)]):
            RectangleRounded(16.0, 3.6, radius=1.2)
    b_ccrown = extrude(s_ccrown.sketch, amount=cap_len)
    loft([s_cbase.sketch, s_ccrown.sketch])

    # Motor cavity (upper half)
    with BuildSketch(plane_yz_motor) as s_ccav:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_ccav.sketch, amount=x_motor_end - x_rear_wall, mode=Mode.SUBTRACT)

    # Rear bulkhead window (upper half)
    with BuildSketch(Plane.YZ.offset(x_wire_end - 0.5)) as s_ctabs:
        with Locations([(0, z_axis)]):
            RectangleRounded(11.0, 8.0, radius=2.0)
    extrude(s_ctabs.sketch, amount=x_rear_wall - x_wire_end + 1.0, mode=Mode.SUBTRACT)

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
            with Locations([(bx, sy * bolt_y, z_axis + 5.5)]):
                Cylinder(radius=6.4 / 2, height=15.0, mode=Mode.SUBTRACT)

cap_solid = cp.part
print(f"Cap built! Volume: {cap_solid.volume:.1f} mm^3")
print("SUCCESS!")
