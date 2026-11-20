# MStation Robotics Workspace Rules

Before modifying any CAD script, STEP/STL mesh, assembly, or slicing/print job in this repository:

1. **Read the Centralized Truth First:** Read [MSTATION_KNOWLEDGE.md](./MSTATION_KNOWLEDGE.md) (and [SPARKX_I7_KNOWLEDGE.md](./SPARKX_I7_KNOWLEDGE.md) for printer/slicing work). Never guess dimensions from memory or leave mating constants out of sync across scripts.
2. **Enforce Core Mechanical Invariants:**
   - **Yoke Gap:** Every dual-shear `MG996R` yoke MUST have `PLUS_Y_IN - MINUS_Y_IN >= 56.0 mm` (canonical `56.50 mm`: `-19.50` to `+37.00`).
   - **Bearing ↔ Fastener Sync:** `623ZZ (3×10×4)` (`Ø10.25 × 4.0 mm` pocket, `1.0 mm` `Ø7.0 mm` lip) MUST ALWAYS pair with **`M3` (`Ø2.60 mm` pilot bore)** on every mating boss/base and `OD <= 5.5 mm` spacer. Never mix `624ZZ (4 mm)` with an `M3` (`Ø2.6 mm`) hole.
   - **Horn Screws:** `-Y` `4.5 mm` arm uses `4× Ø2.20 mm` bores on `Ø14.0 PCD` + `Ø4.6×1.5 mm` outer seat for the included `25T` round plastic horn & servo box screws (`M2`/self-tapping).
   - **Distal Servo Orientation:** Keep servo spline `+X` at the beveled tip (`R_corner <= 20.12 mm`) and wire boot `-X` into a through-height neck slot — verify vertical drop-in `dz = 0..25 mm` == `0.000 mm³`.
   - **Printability & Mesh Health:** Bottom arm bevels `<= 45°`, `0.000 mm³` collision across `0°..±105°`, `min_z = 0.000`, `manifold = yes`, and `number_of_parts = 1` in `OrcaSlicer --info`.
3. **Mandatory Verification Gate:** After any CAD/geometry change, run:
   `python3 .agents/skills/mstation-robotics/scripts/verify_mstation.py`
   and report every check result honestly. Whenever hardware specs or axis designs evolve, update [MSTATION_KNOWLEDGE.md](./MSTATION_KNOWLEDGE.md) in the same change.
