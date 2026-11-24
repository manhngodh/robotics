"""Parametric closed-one-end (blind) knobs for M5 screws/bolts.

Exports two ready-to-print variants:
- m5_knob_threaded.stl / .step: Direct printed M5x0.8 internal blind thread (closed bottom).
- m5_knob_hex.stl / .step: Press-fit blind hex pocket (8.25 mm AF) for an M5 hex bolt head or nut.
"""

import math
import build123d as bd

try:
    from cad import model
except ImportError:
    def model(fn):
        return fn


KNOB_R = 9.0        # Ø18 mm compact outer diameter
KNOB_H = 12.0       # 12 mm total height
FLOOR_H = 3.0       # 3 mm closed-end wall thickness
CHAMFER = 0.8       # 45-degree top/bottom comfort & elephant-foot chamfer
NUM_FLUTES = 6
FLUTE_R = 1.5
FLUTE_OFFSET = 9.6

# M5x0.8 internal thread parameters (FDM-cleared for 0.4 mm nozzle)
M5_PITCH = 0.80
M5_MAJOR_R_NUT = 2.65   # Ø5.30 mm
M5_MINOR_R_NUT = 2.20   # Ø4.40 mm

# M5 hex head / nut pocket (DIN 933 / 934 = 8.0 mm AF + 0.25 mm FDM clearance)
M5_HEX_AF = 8.25


def _make_knob_blank():
    """Create the scalloped Ø28 x 14 mm knob body with 45° top/bottom chamfers."""
    with bd.BuildPart() as p:
        with bd.BuildSketch(bd.Plane.XZ) as sk:
            with bd.BuildLine():
                bd.Polyline(
                    (0.0, 0.0),
                    (KNOB_R - CHAMFER, 0.0),
                    (KNOB_R, CHAMFER),
                    (KNOB_R, KNOB_H - CHAMFER),
                    (KNOB_R - CHAMFER, KNOB_H),
                    (0.0, KNOB_H),
                    (0.0, 0.0),
                )
            bd.make_face()
        bd.revolve(sk.sketch, axis=bd.Axis.Z)

        for k in range(NUM_FLUTES):
            ang = math.radians(k * (360.0 / NUM_FLUTES))
            fx = FLUTE_OFFSET * math.cos(ang)
            fy = FLUTE_OFFSET * math.sin(ang)
            with bd.Locations((fx, fy, KNOB_H / 2.0)):
                bd.Cylinder(radius=FLUTE_R, height=KNOB_H + 1.0, mode=bd.Mode.SUBTRACT)
    return p.part


def _make_trap_thread_cutter(major_r, minor_r, pitch, height):
    """Build a helical trapezoidal thread rib to subtract from a core bore."""
    turns = height / pitch + 2.0
    z_start = -pitch
    helix = bd.Helix(pitch=pitch, height=turns * pitch, radius=minor_r, center=(0, 0, z_start))
    half_base = pitch * 0.40
    half_tip = pitch * 0.16
    inner_r = minor_r - 0.4
    with bd.BuildSketch(bd.Plane.XZ) as sk:
        with bd.BuildLine():
            bd.Polyline(
                (inner_r, z_start - half_base),
                (major_r, z_start - half_tip),
                (major_r, z_start + half_tip),
                (inner_r, z_start + half_base),
                (inner_r, z_start - half_base),
            )
        bd.make_face()
    thread_rib = bd.sweep(sk.sketch, path=helix, is_frenet=True)
    trim_box = bd.Pos(0, 0, height / 2.0) * bd.Box(major_r * 3, major_r * 3, height)
    return thread_rib & trim_box


@model
def m5_knob_threaded():
    """Blind M5x0.8 threaded knob (Ø28 x 14 mm, 3 mm closed end)."""
    blank = _make_knob_blank()
    thread_h = KNOB_H - FLOOR_H  # 11.0 mm (~13.75 threads)

    core_cyl = bd.Pos(0, 0, FLOOR_H + (thread_h + 1.0) / 2.0) * bd.Cylinder(
        radius=M5_MINOR_R_NUT, height=thread_h + 1.0
    )
    tc = bd.Pos(0, 0, FLOOR_H + 0.25) * _make_trap_thread_cutter(
        M5_MAJOR_R_NUT, M5_MINOR_R_NUT, M5_PITCH, thread_h + 0.5
    )
    lead_in = bd.Pos(0, 0, KNOB_H - 0.45) * bd.Cone(
        bottom_radius=M5_MINOR_R_NUT, top_radius=M5_MAJOR_R_NUT + 0.35, height=0.9
    )
    plug = (core_cyl + tc + lead_in).clean()
    part = (blank - plug).clean()
    part.label = "m5_knob_threaded"
    part.color = "#2f6f9f"
    return part


@model
def m5_knob_hex():
    """Blind M5 hex-head / hex-nut press-fit knob (Ø28 x 14 mm, 8.25 mm AF pocket, 3 mm closed end)."""
    blank = _make_knob_blank()
    hex_r = M5_HEX_AF / math.sqrt(3.0)
    pocket_h = KNOB_H - FLOOR_H  # open from z = FLOOR_H to z = KNOB_H

    with bd.BuildPart() as p:
        with bd.BuildSketch(bd.Plane.XY.offset(FLOOR_H)) as sk:
            bd.RegularPolygon(radius=hex_r, side_count=6, rotation=30)
        bd.extrude(sk.sketch, amount=pocket_h + 0.5)
        with bd.Locations((0, 0, KNOB_H - 0.4)):
            bd.Cone(bottom_radius=hex_r * 0.95, top_radius=hex_r + 0.5, height=0.8)

    part = (blank - p.part).clean()
    part.label = "m5_knob_hex"
    part.color = "#4a7c59"
    return part


if __name__ == "__main__":
    for name, builder in [
        ("m5_knob_threaded", m5_knob_threaded),
        ("m5_knob_hex", m5_knob_hex),
    ]:
        shape = builder()
        bb = shape.bounding_box()
        print(
            f"{name}: valid={shape.is_valid}, volume={shape.volume:.1f} mm^3, "
            f"bounds={bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm, "
            f"min_z={bb.min.Z:.3f}, max_z={bb.max.Z:.3f}"
        )
        bd.export_step(shape, f"{name}.step")
        bd.export_stl(shape, f"{name}.stl", tolerance=0.02, angular_tolerance=0.1)
        print(f"  exported {name}.step & {name}.stl")
