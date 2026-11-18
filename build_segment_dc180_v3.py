#!/usr/bin/env python3
"""
Heavy-Duty Rotary Drill End-Effector V3 for Snake Robot Arm.
Solves all stability and retention concerns:
1. Interlocking Side Guide Rails + Rear Backstop (Cap keys into Cradle channel, cannot slide in X, Y, or rotate).
2. Captive 2-End Bulkheads (3.0mm solid rear wall stops backward drilling thrust; 4.0mm solid front faceplate + 2x M2 screws stops forward pull).
3. Flat-oval motor cavity keys motor against rotational torque.
4. Spacious 9.5mm wiring chamber with zip-tie anchor for strain-relief.
5. 4x M3 vertical clamping bolts with captive bottom hex nut pockets.
6. 100% genuine replacement U-fork from Tab 4.
7. Automated push to Onshape & STL/STEP generation.
"""

import os
import sys
import json
import time
from build123d import *
from onshape_client.client import Client

# ====================================================================
# 1. PARAMETERS
# ====================================================================
motor_w = 20.4        # Motor cylindrical diameter [mm]
motor_t = 15.4        # Motor thickness between flats [mm]
motor_l = 32.0        # Motor metal body length [mm]
clearance = 0.25      # Fit tolerance [mm]

r_c = (motor_w + 2 * clearance) / 2  # 10.45 mm
h_c = (motor_t + 2 * clearance) / 2  # 7.95 mm

z_axis = 15.0         # Spindle centerline [mm]
x_start = 19.5        # Tab 4 U-fork bridge interface [mm]
x_wire_end = 29.0     # End of wire chamber / Start of cap & rear bulkhead [mm]
x_rear_wall = 32.0    # End of rear bulkhead / Start of motor cavity [mm]
x_motor_end = 64.2    # End of motor cavity / Start of front faceplate [mm]
x_end = 68.2          # End of front faceplate [mm]

body_w = 34.0         # Cradle outer width across ears [mm]
pocket_w = 29.8       # Width between cradle guide rails [mm]
cap_w = 29.4          # Cap width (0.2mm clearance per side) [mm]
rail_h = 2.5          # Guide rail height above split plane (Z = 17.5 mm) [mm]

bolt_y = 12.5         # M3 bolt Y position (+/- 12.5 mm)
bolt_x1 = 39.0        # Rear M3 bolt pair [mm]
bolt_x2 = 57.0        # Front M3 bolt pair [mm]

out_dir = "/Users/manhngodh/development/robotics"
os.makedirs(out_dir, exist_ok=True)

# ====================================================================
# 2. LOAD REPLACEMENT U-FORK TAB 4
# ====================================================================
print("1. Loading replacement U-fork from Tab 4...")
tab4_step = "/Users/manhngodh/development/onshape/02_universal_segment_tab4.step"
part = import_step(tab4_step)
fork = part.solids()[0].fuse(part.solids()[1]).fuse(part.solids()[3])
print(f"   U-fork fused! Volume: {fork.volume:.1f} mm^3")

# ====================================================================
# 3. BUILD MONOLITHIC CRADLE WITH GUIDE RAILS & BULKHEADS
# ====================================================================
print("2. Building Cradle with Interlocking Guide Rails & Captive Bulkheads...")
with BuildPart() as cr:
    # Outer hull: height up to Z = 17.5mm (z_axis + rail_h) to form side guide rails
    with BuildSketch(Plane.YZ.offset(x_start)) as s_cradle:
        with Locations([(0, (z_axis + rail_h) / 2)]):
            RectangleRounded(body_w, z_axis + rail_h, radius=2.5)
    extrude(s_cradle.sketch, amount=x_end - x_start)

    # Pocket channel for cap between guide rails:
    # Cuts Y in [-14.9, 14.9] from Z = 15.0 up to 18.0 along X in [x_wire_end, x_end + 1.0]
    with BuildSketch(Plane.XY.offset(z_axis)) as s_pocket:
        with Locations([((x_wire_end + x_end + 1.0)/2, 0)]):
            Rectangle(x_end + 1.0 - x_wire_end, pocket_w)
    extrude(s_pocket.sketch, amount=rail_h + 1.0, mode=Mode.SUBTRACT)

    # Motor cavity (lower half) from x_rear_wall to x_motor_end
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

    # Zip-tie anchor slot (through-tunnel for zip-tie under wire bed at X = 24.25)
    with BuildSketch(Plane.XZ.offset(-8.5)) as s_ziptie:
        with Locations([(24.25, 1.8)]):
            RectangleRounded(3.2, 2.0, radius=0.6)
    extrude(s_ziptie.sketch, amount=17.0, mode=Mode.SUBTRACT)

    # Rear bulkhead window (X in [29.0, 32.0]): 11x8mm for tabs and rear boss
    with BuildSketch(Plane.YZ.offset(x_wire_end - 0.5)) as s_tabs:
        with Locations([(0, z_axis)]):
            RectangleRounded(11.0, 8.0, radius=2.0)
    extrude(s_tabs.sketch, amount=x_rear_wall - x_wire_end + 1.0, mode=Mode.SUBTRACT)

    # Front spindle faceplate cuts (lower half):
    # 1. Motor collar saddle Ø6.8mm
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_col:
        with Locations([(0, z_axis)]):
            Circle(radius=6.8 / 2)
    extrude(s_col.sketch, amount=2.0, mode=Mode.SUBTRACT)

    # 2. Front collet counterbore Ø12.5mm
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

    # 4. 4x M3 clamping bolt holes & bottom captive hex nut pockets
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, -1.0)]):
                Cylinder(radius=3.4 / 2, height=z_axis + 5.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, 0)]):
                with BuildSketch() as s_nut:
                    RegularPolygon(radius=5.85 / (3**0.5), side_count=6)
                extrude(s_nut.sketch, amount=3.5, mode=Mode.SUBTRACT)

    # 5. Bottom cooling vent under motor
    with BuildSketch(Plane.XY.offset(-1.0)) as s_vent:
        with Locations([((x_rear_wall + x_motor_end)/2, 0)]):
            RectangleRounded(18.0, 7.0, radius=2.0)
    extrude(s_vent.sketch, amount=z_axis + 2.0, mode=Mode.SUBTRACT)

cradle = fork.fuse(cr.part)
print(f"   Monolithic Cradle built! Volume: {cradle.volume:.1f} mm^3")

# ====================================================================
# 4. BUILD INTERLOCKING CLAMP CAP
# ====================================================================
print("3. Building Interlocking Clamp Cap (Sliding Keyway into Guide Rails)...")
cap_len = x_end - x_wire_end  # 39.2 mm

with BuildPart() as cp:
    # Cap profile sketch: Base ears + Upper crown
    with BuildSketch(Plane.YZ.offset(x_wire_end)) as s_cap_profile:
        with Locations([(0, z_axis + 5.5 / 2)]):
            RectangleRounded(cap_w, 5.5, radius=1.5)
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

    # Front collar saddle Ø6.8mm (upper half)
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_ccol:
        with Locations([(0, z_axis)]):
            Circle(radius=6.8 / 2)
    extrude(s_ccol.sketch, amount=2.0, mode=Mode.SUBTRACT)

    # Front collet counterbore Ø12.5mm (upper half)
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
print(f"   Interlocking Cap built! Volume: {cap.volume:.1f} mm^3")

# ====================================================================
# 5. MODEL MOTOR, CHUCK, DRILL BIT, FASTENERS & WIRING DUMMIES
# ====================================================================
print("4. Modeling internal components (DC 180, Collet Chuck, Drill Bit, Fasteners, Wiring)...")

# Motor DC 180 Dummy
with BuildPart() as motor_b:
    with BuildSketch(plane_yz_motor) as s_m:
        Circle(radius=20.4 / 2)
        Rectangle(25, 15.4, mode=Mode.INTERSECT)
    extrude(s_m.sketch, amount=32.0)
    # Front collar (Ø6.0 x 1.8mm)
    with BuildSketch(Plane.YZ.offset(x_motor_end)) as s_col:
        with Locations([(0, z_axis)]):
            Circle(radius=6.0 / 2)
    extrude(s_col.sketch, amount=1.8)
    # Front shaft (Ø2.0 x 10mm)
    with BuildSketch(Plane.YZ.offset(x_motor_end)) as s_sh:
        with Locations([(0, z_axis)]):
            Circle(radius=2.0 / 2)
    extrude(s_sh.sketch, amount=10.0)
    # Rear boss (Ø6.0 x 1.5mm)
    with BuildSketch(Plane.YZ.offset(x_rear_wall - 1.5)) as s_rb:
        with Locations([(0, z_axis)]):
            Circle(radius=6.0 / 2)
    extrude(s_rb.sketch, amount=1.5)
    # 2x Copper solder tabs
    for sy in [-1, 1]:
        with Locations([(x_rear_wall - 4.5, sy * 5.2, z_axis)]):
            Box(4.5, 2.0, 1.2)
motor_part = motor_b.part

# Brass Collet Chuck (Ø8.5 x 15mm)
with BuildPart() as chuck_b:
    with BuildSketch(Plane.YZ.offset(x_end + 1.0)) as s_ck:
        with Locations([(0, z_axis)]):
            Circle(radius=8.5 / 2)
    extrude(s_ck.sketch, amount=13.0)
    with BuildSketch(Plane.YZ.offset(x_end + 14.0)) as s_tn:
        with Locations([(0, z_axis)]):
            Circle(radius=6.0 / 2)
    extrude(s_tn.sketch, amount=4.0)
chuck_part = chuck_b.part

# Drill Bit (Ø2.0 x 22mm)
with BuildPart() as drill_b:
    with BuildSketch(Plane.YZ.offset(x_end + 18.0)) as s_db:
        with Locations([(0, z_axis)]):
            Circle(radius=2.0 / 2)
    extrude(s_db.sketch, amount=18.0)
    with Locations([(x_end + 36.0, 0, z_axis)]):
        Cone(bottom_radius=2.0 / 2, top_radius=0.1, height=2.5, rotation=(0, 90, 0))
drill_part = drill_b.part

# Fasteners (4x M3 + 4x Hex Nuts + 2x M2 front screws)
with BuildPart() as fast_b:
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, 19.0)]):
                Cylinder(radius=5.5 / 2, height=3.0)
            with Locations([(bx, sy * bolt_y, 2.0)]):
                Cylinder(radius=3.0 / 2, height=17.0)
            with Locations([(bx, sy * bolt_y, 0.0)]):
                with BuildSketch() as s_n:
                    RegularPolygon(radius=5.5 / (3**0.5), side_count=6)
                extrude(s_n.sketch, amount=2.4)
    for sy in [-1, 1]:
        with Locations([(x_end - 1.5, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_m2h:
                Circle(radius=3.8 / 2)
            extrude(s_m2h.sketch, amount=2.0)
        with Locations([(x_motor_end - 2.0, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_m2s:
                Circle(radius=2.0 / 2)
            extrude(s_m2s.sketch, amount=6.0)
fasteners_part = fast_b.part

# Silicone Wires & Zip-Tie Dummy
with BuildPart() as wire_b:
    for sy in [-1, 1]:
        with Locations([( (27.0 + 16.0)/2, sy * 4.0, z_axis - 4.5 )]):
            Box(11.0, 2.2, 2.2)
    with Locations([(24.25, 0, z_axis - 4.5)]):
        Box(2.5, 12.0, 4.0)
wire_part = wire_b.part

# ====================================================================
# 6. EXPORT STEP & STL FILES
# ====================================================================
print("5. Exporting STEP and STL files...")
step_cradle_path = os.path.join(out_dir, "02_segment_dc180_v3_cradle.step")
step_cap_path = os.path.join(out_dir, "02_segment_dc180_v3_cap.step")
step_asm_path = os.path.join(out_dir, "02_segment_dc180_v3_assembly.step")
step_full_path = os.path.join(out_dir, "02_segment_dc180_v3_complete_with_motor.step")

export_step(cradle, step_cradle_path)
export_step(cap, step_cap_path)
export_step(Compound([cradle, cap]), step_asm_path)
export_step(Compound([cradle, cap, motor_part, chuck_part, drill_part, fasteners_part, wire_part]), step_full_path)

stl_cradle_path = os.path.join(out_dir, "02_segment_dc180_v3_cradle.stl")
stl_cap_path = os.path.join(out_dir, "02_segment_dc180_v3_cap.stl")
export_stl(cradle, stl_cradle_path)
export_stl(cap, stl_cap_path)

# Build plate side-by-side layout: Cradle flat at Z=0, Cap rotated 180 deg placed next to it
cap_top_z = z_axis + 11.0
cap_bed_loc = Location((48.0, 32.0, cap_top_z), (180, 0, 0))
cap_on_bed = cap.moved(cap_bed_loc)
plate_compound = Compound([cradle, cap_on_bed])
stl_plate_path = os.path.join(out_dir, "02_segment_dc180_v3_plate.stl")
export_stl(plate_compound, stl_plate_path)
print("   All STEP & STL files saved successfully!")

# Export Cutaway STL parts for photorealistic OpenSCAD render
with BuildPart() as cut_box_b:
    with Locations([(25.0, 25.0, 15.0)]):
        Box(120.0, 50.0, 50.0)
cut_box = cut_box_b.part

cradle_cut = cradle.cut(cut_box)
cap_cut = cap.cut(cut_box)

export_stl(cradle_cut, os.path.join(out_dir, "render_v3_cradle_cutaway.stl"))
export_stl(cap_cut, os.path.join(out_dir, "render_v3_cap_cutaway.stl"))
export_stl(motor_part, os.path.join(out_dir, "render_v3_motor.stl"))
export_stl(chuck_part, os.path.join(out_dir, "render_v3_chuck.stl"))
export_stl(drill_part, os.path.join(out_dir, "render_v3_drill.stl"))
export_stl(fasteners_part, os.path.join(out_dir, "render_v3_fasteners.stl"))
export_stl(wire_part, os.path.join(out_dir, "render_v3_wire.stl"))

# ====================================================================
# 7. PUSH TO ONSHAPE
# ====================================================================
print("6. Pushing V3 complete assembly to Onshape document 554c3f4204fffe9cc429ac54...")
try:
    with open(os.path.expanduser("~/.onshape/credentials.json")) as f:
        creds = json.load(f)
    client = Client(configuration={
        "base_url": creds.get("baseUrl", "https://cad.onshape.com"),
        "access_key": creds["accessKey"],
        "secret_key": creds["secretKey"]
    })

    did = "554c3f4204fffe9cc429ac54"
    wid = "a47520ea45b388137ec9a49b"

    with open(step_full_path, "rb") as f:
        res = client.blob_elements_api.upload_file_create_element(
            did=did,
            wid=wid,
            file=f,
            translate=True,
            format_name="STEP",
            unit="millimeter",
            _preload_content=False
        )
    data = json.loads(res.data.decode("utf-8"))
    tid = data.get("translationId")
    print(f"   Upload OK! Translation ID: {tid}")

    if tid:
        print("   Waiting for translation to finish...")
        for _ in range(35):
            t_res = client.translation_api.get_translation(tid=tid, _preload_content=False)
            t_data = json.loads(t_res.data.decode("utf-8"))
            if t_data.get("requestState") == "DONE":
                elems = t_data.get("resultElementIds", [])
                eid = elems[0] if elems else ""
                print(f"   Translation DONE! New Part Studio element: {eid}")
                direct_url = f"https://cad.onshape.com/documents/{did}/w/{wid}/e/{eid}"
                print(f"   >>> Direct 3D Part Studio URL: {direct_url}")
                # Save element ID
                with open(os.path.join(out_dir, "v3_element_id.txt"), "w") as ef:
                    ef.write(eid)
                break
            time.sleep(1)
except Exception as ex:
    print("   Onshape push error:", ex)

print("\nAll V3 tasks completed!")
