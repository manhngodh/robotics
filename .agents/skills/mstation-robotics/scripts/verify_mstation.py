#!/usr/bin/env python3
"""
MStation Centralized Design & Pre-Print Gate Verifier.
Verifies cross-part mechanical invariants across the 6-axis CAD suite:
  1. Yoke inner gap >= 56.0 mm and parameter sync across links
  2. Bearing pocket <-> mating bolt/pilot consistency (624ZZ 4mm ID <-> M4 Ø3.4mm)
  3. Distal servo orientation (shaft at +X outer tip) & vertical drop-in clearance
  4. Real MG90S cavity clearance in MG996R->MG90S 2-arm cover
  5. Arm bottom bevel overhang <= 45 deg
  6. OrcaSlicer STL health (manifold=yes, number_of_parts=1, min_z=0.000)
"""

import os
import re
import subprocess
import sys

DIR = "/Users/manhngodh/development/robotics"
ORCA = "/Applications/OrcaSlicer.app/Contents/MacOS/OrcaSlicer"


def check(label, cond, detail):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {label}: {detail}")
    return cond


def main():
    sys.path.insert(0, DIR)
    ok = True

    import build_mg996r_to_mg996r as jlink
    import build_mg996r_to_mg90s_2arm as cov2
    from build123d import Box, Compound, Pos, Solid, import_step

    print("=== 1. Cross-Part Yoke & Link Sync ===")
    gap_jl = jlink.PLUS_Y_IN - jlink.MINUS_Y_IN
    gap_2a = cov2.PLUS_Y_IN - cov2.MINUS_Y_IN
    ok &= check("Join Link yoke gap >= 56.0 mm", gap_jl >= 56.0, f"{gap_jl:.2f} mm ({jlink.MINUS_Y_IN}..{jlink.PLUS_Y_IN})")
    ok &= check("2-Arm 96r->90s yoke gap >= 56.0 mm", gap_2a >= 56.0, f"{gap_2a:.2f} mm ({cov2.MINUS_Y_IN}..{cov2.PLUS_Y_IN})")
    ok &= check(
        "Yoke sync (Join Link == 2-Arm)",
        abs(jlink.MINUS_Y_IN - cov2.MINUS_Y_IN) < 1e-6 and abs(jlink.PLUS_Y_IN - cov2.PLUS_Y_IN) < 1e-6,
        f"JoinLink=({jlink.MINUS_Y_IN},{jlink.PLUS_Y_IN}) vs 2Arm=({cov2.MINUS_Y_IN},{cov2.PLUS_Y_IN})",
    )
    ok &= check("Join Link BRIDGE_X_MIN >= 17.5 mm", jlink.BRIDGE_X_MIN >= 17.5, f"{jlink.BRIDGE_X_MIN:.2f} mm")
    ok &= check("2-Arm BRIDGE_X_MIN >= 16.5 mm", cov2.BRIDGE_X_MIN >= 16.5, f"{cov2.BRIDGE_X_MIN:.2f} mm")

    print("\n=== 2. Bearing (623ZZ 3x10x4 M3) <-> Mating Pilot Bore Sync ===")
    with open(os.path.join(DIR, "build_arm_base_v6.py")) as f:
        base_src = f.read()
    m_base = re.search(r"Cylinder\(radius=([0-9.]+),\s*height=\(HEAD_X_BACK\s*\+\s*BEARING_BOSS_LEN", base_src)
    base_bore_d = float(m_base.group(1)) * 2.0 if m_base else 0.0
    ok &= check(
        "Joint Base V6 +X spigot M3 bore (2.40..2.70 mm for 623ZZ 3mm ID)",
        2.4 <= base_bore_d <= 2.7,
        f"found Ø{base_bore_d:.2f} mm in build_arm_base_v6.py",
    )

    with open(os.path.join(DIR, "build_mg996r_to_mg996r.py")) as f:
        jl_src = f.read()
    m_jl = re.search(r"# F\. Coaxial M3 pilot bore[\s\S]*?Cylinder\(radius=([0-9.]+)", jl_src)
    jl_bore_d = float(m_jl.group(1)) * 2.0 if m_jl else 0.0
    ok &= check("Join Link V6 underside M3 pilot bore (2.40..2.70 mm)", 2.4 <= jl_bore_d <= 2.7, f"found Ø{jl_bore_d:.2f} mm")
    ok &= check("Join Link +Y 623ZZ pocket Ø10.25 mm (radius=5.125)", "radius=5.125" in jl_src, "radius=5.125")

    with open(os.path.join(DIR, "build_mg996r_to_mg90s_2arm.py")) as f:
        c2_src = f.read()
    ok &= check("2-Arm +Y 623ZZ pocket Ø10.25 mm (radius=5.125)", "radius=5.125" in c2_src, "radius=5.125")

    print("\n=== 3. Distal Servo Orientation & Vertical Drop-In ===")
    ok &= check(
        "Join Link MG996R #2 shaft at +X outer tip",
        jlink.J2_SHAFT_X > jlink.J2_BODY_CX and (jlink.NOSE_TIP_X - jlink.J2_SHAFT_X) <= 19.0,
        f"J2_SHAFT_X={jlink.J2_SHAFT_X}, J2_BODY_CX={jlink.J2_BODY_CX}, NOSE_TIP_X={jlink.NOSE_TIP_X}",
    )
    l1 = import_step(os.path.join(DIR, "arm_link_mg996r_to_mg996r.step"))
    m2 = jlink.build_virtual_mg996r2()
    max_dz_clash = 0.0
    for dz in (0, 2, 5, 10, 15, 20, 25):
        inter = l1.intersect(m2.translate((0, 0, dz)))
        v = sum(s.volume for s in inter.solids()) if inter else 0.0
        max_dz_clash = max(max_dz_clash, v)
    ok &= check("MG996R #2 vertical drop-in (dz=0..25 mm)", max_dz_clash < 1e-3, f"max clash={max_dz_clash:.3f} mm3")

    print("\n=== 4. MG90S Cavity Freedom & Print Bevel Check ===")
    orig = import_step(os.path.join(DIR, "u_break_cover_orig.step"))
    oh_raw = Compound(orig.intersect(Pos(38.0, 0.0, 11.0) * Box(46.0, 21.0, 24.0)))
    oh = Solid(max(oh_raw.shells(), key=lambda s: s.area)).translate((cov2.HEAD_SHIFT_X, cov2.HEAD_SHIFT_Y, 0.0))
    p2a = import_step(os.path.join(DIR, "arm_link_mg996r_to_mg90s_2arm.step"))
    pocket_diffs = []
    for x in range(18, 63, 2):
        for z in (2, 6, 11, 16, 20):
            v_oh = Compound(oh.intersect(Pos(x, cov2.HEAD_SHIFT_Y, z) * Box(0.5, 8.0, 0.5))).volume
            v_2a = Compound(p2a.intersect(Pos(x, cov2.HEAD_SHIFT_Y, z) * Box(0.5, 8.0, 0.5))).volume
            if abs(v_2a - v_oh) > 0.01:
                pocket_diffs.append((x, z))
    ok &= check("2-Arm MG90S pocket unobstructed", len(pocket_diffs) == 0, f"diffs={pocket_diffs}")

    with open(os.path.join(DIR, "build_mg996r_to_mg90s_2arm.py")) as f:
        c2_src = f.read()
    has_bevel_45 = "(-2.0, 0.0)" in c2_src
    ok &= check("2-Arm bottom lug bevel <= 45 deg ((-2.0, 0.0))", has_bevel_45, "(-2.0, 0.0) present" if has_bevel_45 else "missing (-2.0, 0.0)")

    print("\n=== 5. Printable STL Mesh Health (OrcaSlicer --info) ===")
    if os.path.exists(ORCA):
        for stl in ("PrintReady_MG996R_to_MG996R.stl", "PrintReady_MG996R_to_MG90S_2Arm.stl"):
            path = os.path.join(DIR, stl)
            res = subprocess.run([ORCA, "--info", path], capture_output=True, text=True, check=False)
            out = res.stdout + res.stderr
            mani = "manifold = yes" in out
            parts_m = re.search(r"number_of_parts\s*=\s*(\d+)", out)
            parts = int(parts_m.group(1)) if parts_m else -1
            minz_m = re.search(r"min_z\s*=\s*([0-9.-]+)", out)
            minz = float(minz_m.group(1)) if minz_m else -1.0
            ok &= check(f"{stl} manifold & single shell & Z=0", mani and parts == 1 and abs(minz) < 0.01, f"manifold={mani}, parts={parts}, min_z={minz}")
    else:
        print("[SKIP] OrcaSlicer not found at", ORCA)

    print("\nOverall Gate:", "ALL PASSED" if ok else "FAILED (see items above)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
