"""Parametric closed-one-end (blind) M5.5 knob (Ø5.50 mm major, M5x0.8 pitch).

Compact Ø19 x 12 mm body with 3 mm closed end and 9 mm blind thread depth.
"""

import math
import build123d as bd

try:
    from cad import model
except ImportError:
    def model(fn):
        return fn


KNOB_R = 9.5        # Ø19 mm compact outer diameter
KNOB_H = 12.0       # 12 mm total height
FLOOR_H = 3.0       # 3 mm closed-end wall thickness
CHAMFER = 0.8       # 45-degree top/bottom comfort & elephant-foot chamfer
NUM_FLUTES = 6
FLUTE_R = 1.5
FLUTE_OFFSET = 10.1

# M5.5 threaded bore (Ø5.50 mm major, Ø4.60 mm minor, 0.80 mm pitch for M5 screw FDM fit)
M5_5_PITCH = 0.80
M5_5_MAJOR_R_NUT = 2.75   # Ø5.50 mm (+0.25 mm radial clearance over nominal M5)
M5_5_MINOR_R_NUT = 2.30   # Ø4.60 mm (+0.23 mm radial clearance over nominal M5)


def _make_knob_blank():
    """Create the scalloped Ø19 x 12 mm knob body with 45° top/bottom chamfers."""
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
def m5_5_knob_threaded():
    """Blind M5.5 threaded knob (Ø19 x 12 mm, 3 mm closed end)."""
    blank = _make_knob_blank()
    thread_h = KNOB_H - FLOOR_H  # 9.0 mm (~11.25 threads)

    core_cyl = bd.Pos(0, 0, FLOOR_H + (thread_h + 1.0) / 2.0) * bd.Cylinder(
        radius=M5_5_MINOR_R_NUT, height=thread_h + 1.0
    )
    tc = bd.Pos(0, 0, FLOOR_H + 0.25) * _make_trap_thread_cutter(
        M5_5_MAJOR_R_NUT, M5_5_MINOR_R_NUT, M5_5_PITCH, thread_h + 0.5
    )
    lead_in = bd.Pos(0, 0, KNOB_H - 0.50) * bd.Cone(
        bottom_radius=M5_5_MINOR_R_NUT, top_radius=M5_5_MAJOR_R_NUT + 0.40, height=1.0
    )
    plug = (core_cyl + tc + lead_in).clean()
    part = (blank - plug).clean()
    part.label = "m5_5_knob_threaded"
    part.color = "#2f6f9f"
    return part


if __name__ == "__main__":
    shape = m5_5_knob_threaded()
    bb = shape.bounding_box()
    print(
        f"m5_5_knob_threaded: valid={shape.is_valid}, volume={shape.volume:.1f} mm^3, "
        f"bounds={bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm, "
        f"min_z={bb.min.Z:.3f}, max_z={bb.max.Z:.3f}"
    )
    bd.export_step(shape, "m5_5_knob_threaded.step")
    bd.export_stl(shape, "m5_5_knob_threaded.stl", tolerance=0.02, angular_tolerance=0.1)
    print("  exported m5_5_knob_threaded.step & m5_5_knob_threaded.stl")
