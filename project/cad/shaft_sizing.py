"""
First-pass mechanical sizing for the crankshaft/crank-pin, to hand off
as concrete dimensions into CAD (Siemens NX).

Inputs are pulled from the results of sizing-scripts/crank_slider_sizing.py
and models/digital_twin_transient.py -- this script does NOT recompute
the torque/pressure cycle, it just turns the peak loads already found
into a first-pass shaft diameter and crank-pin load spec.

KNOWN LIMITATIONS (explicitly flagged, not fixed here):
  - Main shaft sizing is TORSION ONLY. A real crankshaft also sees
    bending from the crank throw offset -- combined bending+torsion
    (e.g. ASME B106.1M shaft design method) should be checked before
    this is a final dimension, not just a first pass.
  - Static safety factor only -- no fatigue analysis (Soderberg/Goodman),
    despite the load being fully cyclic. The torque and gas force both
    swing from near-zero to peak every revolution, which is a genuine
    fatigue-loading case for the material.
  - 316L yield strength (205 MPa) is a typical annealed-condition
    handbook value, not a certified material datasheet figure.
"""

import numpy as np

# ---------------------------------------------------------------------
# Inputs from prior results (see docstring)
# ---------------------------------------------------------------------
T_peak = 156.0        # N*m, peak shaft torque (digital_twin_transient.py)
p2 = 200e5            # Pa, discharge pressure (design-basis.md)
D_bore = 0.030        # m, cylinder bore
r_crank = 0.015       # m, crank radius (design-basis.md)

A_piston = np.pi / 4 * D_bore**2
F_gas_peak = p2 * A_piston  # N, peak gas force -> peak crank-pin radial load

# ---------------------------------------------------------------------
# Material: 316L stainless steel (H2-compatible, per design-basis.md Sec.2)
# ---------------------------------------------------------------------
Sy = 205e6  # Pa, yield strength, annealed 316L (typical handbook value --
             # replace with a certified datasheet value before finalising)

# ---------------------------------------------------------------------
# Main shaft diameter -- torsion only, first pass
# ---------------------------------------------------------------------
SF = 3.0  # safety factor: covers keyway stress concentration, fatigue
           # derating (approximate), and general first-pass conservatism.
           # NOT a substitute for a real fatigue check (see limitations above).
tau_allow = 0.5 * Sy / SF  # max-shear-stress theory
d_shaft_min = (16 * T_peak / (np.pi * tau_allow)) ** (1/3)
d_shaft_std = np.ceil(d_shaft_min * 1000 / 2) * 2  # round up to even mm

print("=== Crankshaft / crank-pin first-pass sizing (torsion only) ===")
print(f"Peak shaft torque:            {T_peak:.1f} N*m")
print(f"Peak gas force (crank pin):   {F_gas_peak:.0f} N ({F_gas_peak/1000:.2f} kN)")
print(f"Material: 316L stainless, Sy = {Sy/1e6:.0f} MPa")
print(f"Allowable shear stress (SF={SF}): {tau_allow/1e6:.1f} MPa")
print(f"Minimum main shaft diameter:  {d_shaft_min*1000:.1f} mm")
print(f"Recommended (rounded) diameter: {d_shaft_std:.0f} mm")
print()
print("NEXT STEPS before this is CAD-final, not just a starting sketch:")
print("  1. Combined bending+torsion check (ASME B106.1M or equivalent)")
print("  2. Fatigue check (Soderberg/Goodman) -- load is fully cyclic")
print("  3. Crank-pin bearing selection sized against the 14.14 kN peak radial load")
print("  4. Confirm Sy against an actual 316L material datasheet")
