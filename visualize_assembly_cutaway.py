#!/usr/bin/env python3
"""
Generate Complete 3D Assembly with DC 180 Motor, Brass Collet Chuck,
Drill Bit, Fasteners, Wiring, and Exploded/Cutaway Visualization.
Exports STEP to Onshape and renders explanatory 3D diagrams.
"""

import os
import json
import time
from build123d import *
from onshape_client.client import Client

# ====================================================================
# 1. LOAD CRADLE & CAP
# ====================================================================
out_dir = "/Users/manhngodh/development/robotics"
cradle = import_step(os.path.join(out_dir, "02_segment_dc180_cradle.step"))
cap = import_step(os.path.join(out_dir, "02_segment_dc180_cap.step"))

z_axis = 15.0
x_motor_start = 30.0
x_motor_end = 62.0
x_end = 66.0
bolt_y = 13.5
bolt_x1 = 38.0
bolt_x2 = 55.0

# ====================================================================
# 2. MODEL REALISTIC DC 180 MOTOR
# ====================================================================
print("1. Modeling DC 180 motor dummy...")
with BuildPart() as motor_b:
    # Metal casing (32mm)
    plane_yz_motor = Plane(origin=(x_motor_start, 0, z_axis), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    with BuildSketch(plane_yz_motor) as s_m:
        Circle(radius=20.4 / 2)
        Rectangle(25, 15.4, mode=Mode.INTERSECT)
    extrude(s_m.sketch, amount=32.0)

    # Front collar (Ø6.1 x 1.8mm)
    with BuildSketch(Plane.YZ.offset(x_motor_end)) as s_col:
        with Locations([(0, z_axis)]):
            Circle(radius=6.1 / 2)
    extrude(s_col.sketch, amount=1.8)

    # Front shaft (Ø2.0 x 10mm)
    with BuildSketch(Plane.YZ.offset(x_motor_end)) as s_sh:
        with Locations([(0, z_axis)]):
            Circle(radius=2.0 / 2)
    extrude(s_sh.sketch, amount=10.0)

    # Rear bearing boss (Ø6.0 x 1.5mm)
    with BuildSketch(Plane.YZ.offset(x_motor_start - 1.5)) as s_rb:
        with Locations([(0, z_axis)]):
            Circle(radius=6.0 / 2)
    extrude(s_rb.sketch, amount=1.5)

    # 2x Rear copper solder tabs
    for sy in [-1, 1]:
        with Locations([(x_motor_start - 4.5, sy * 5.5, z_axis)]):
            Box(4.5, 2.0, 1.2)

motor_part = motor_b.part

# ====================================================================
# 3. MODEL BRASS COLLET CHUCK & DRILL BIT
# ====================================================================
print("2. Modeling Collet Chuck & Drill Bit...")
# Brass collet body (Ø8.5 x 15mm)
with BuildPart() as chuck_b:
    with BuildSketch(Plane.YZ.offset(x_end + 1.0)) as s_ck:
        with Locations([(0, z_axis)]):
            Circle(radius=8.5 / 2)
    extrude(s_ck.sketch, amount=13.0)
    # Tapered nose
    with BuildSketch(Plane.YZ.offset(x_end + 14.0)) as s_tn:
        with Locations([(0, z_axis)]):
            Circle(radius=6.0 / 2)
    extrude(s_tn.sketch, amount=4.0)

chuck_part = chuck_b.part

# Drill bit (Ø2.0 x 22mm, sticks out 15mm beyond chuck)
with BuildPart() as drill_b:
    with BuildSketch(Plane.YZ.offset(x_end + 18.0)) as s_db:
        with Locations([(0, z_axis)]):
            Circle(radius=2.0 / 2)
    extrude(s_db.sketch, amount=18.0)
    with Locations([(x_end + 36.0, 0, z_axis)]):
        Cone(bottom_radius=2.0 / 2, top_radius=0.1, height=2.5, rotation=(0, 90, 0))

drill_part = drill_b.part

# ====================================================================
# 4. MODEL FASTENERS (4x M3 + 2x M2)
# ====================================================================
print("3. Modeling fasteners...")
with BuildPart() as fast_b:
    # 4x M3 socket head screws (heads at Z = 22.0, shaft down to Z = 0)
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, 20.0)]):
                Cylinder(radius=5.5 / 2, height=3.0)  # Head
            with Locations([(bx, sy * bolt_y, 2.0)]):
                Cylinder(radius=3.0 / 2, height=18.0) # Shaft
            with Locations([(bx, sy * bolt_y, 0.0)]):
                with BuildSketch() as s_n:
                    RegularPolygon(radius=5.5 / (3**0.5), side_count=6)
                extrude(s_n.sketch, amount=2.4)       # Hex nut

    # 2x M2 front screws (heads at X = 66.0, shafts entering motor at X = 62.0)
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

# ====================================================================
# 5. MODEL SILICONE WIRES & ZIP-TIE
# ====================================================================
print("4. Modeling wiring & zip-tie...")
with BuildPart() as wire_b:
    # 2 wires routing from solder tabs at X=26.0 through chamber to bridge at X=16.0
    for sy, color_offset in [(-1, -1.8), (1, 1.8)]:
        with Locations([( (26.0 + 16.0)/2, sy * 4.0, z_axis - 4.0 )]):
            Box(12.0, 2.2, 2.2)
    # Zip-tie strap wrapping wires at X = 24.5
    with Locations([(24.5, 0, z_axis - 4.0)]):
        Box(2.5, 12.0, 4.0)

wire_part = wire_b.part

# ====================================================================
# 6. EXPORT COMPLETE ASSEMBLED STEP & PUSH TO ONSHAPE
# ====================================================================
print("5. Exporting complete assembly STEP...")
full_assembly_compound = Compound([
    cradle,
    cap,
    motor_part,
    chuck_part,
    drill_part,
    fasteners_part,
    wire_part
])

asm_step_path = os.path.join(out_dir, "02_segment_dc180_complete_with_motor.step")
export_step(full_assembly_compound, asm_step_path)
print(f"   Saved {asm_step_path}")

print("6. Pushing complete assembly to Onshape document 554c3f4204fffe9cc429ac54...")
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

    with open(asm_step_path, "rb") as f:
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
        for _ in range(30):
            t_res = client.translation_api.get_translation(tid=tid, _preload_content=False)
            t_data = json.loads(t_res.data.decode("utf-8"))
            if t_data.get("requestState") == "DONE":
                elems = t_data.get("resultElementIds", [])
                eid = elems[0] if elems else ""
                print(f"   Translation DONE! Element ID: {eid}")
                print(f"   Direct Onshape URL: https://cad.onshape.com/documents/{did}/w/{wid}/e/{eid}")
                break
            time.sleep(1)
except Exception as ex:
    print("   Onshape push error:", ex)

# ====================================================================
# 7. EXPORT CUTAWAY (HALF-SECTION) FOR RENDER
# ====================================================================
print("7. Exporting half-cutaway for visualization...")
# Cut away Y > 0 on cradle and cap so we can see inside!
with BuildPart() as cut_box_b:
    with Locations([(25.0, 25.0, 15.0)]):
        Box(120.0, 50.0, 50.0)
cut_box = cut_box_b.part

cradle_cut = cradle.cut(cut_box)
cap_cut = cap.cut(cut_box)

export_stl(cradle_cut, os.path.join(out_dir, "render_cradle_cutaway.stl"))
export_stl(cap_cut, os.path.join(out_dir, "render_cap_cutaway.stl"))
export_stl(motor_part, os.path.join(out_dir, "render_motor.stl"))
export_stl(chuck_part, os.path.join(out_dir, "render_chuck.stl"))
export_stl(drill_part, os.path.join(out_dir, "render_drill.stl"))
export_stl(fasteners_part, os.path.join(out_dir, "render_fasteners.stl"))
export_stl(wire_part, os.path.join(out_dir, "render_wire.stl"))
print("Done all exports!")
