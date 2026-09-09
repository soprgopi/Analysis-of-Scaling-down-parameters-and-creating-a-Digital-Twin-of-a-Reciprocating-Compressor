# Phase 2 Conclusion: Digital Twin — Electric Drive + Mechanical Drivetrain + Gas Load

**Status:** Concluded
**Date:** 2026-09-09

## Objective

Determine whether a mechanical crankshaft-drive, proven on large-scale
high-speed (100-2000 RPM) reciprocating gas compressors, can be adapted
to a small-scale, low-speed (1-30 RPM) hydrogen compressor — and if so,
what that adaptation actually requires. carried forward as
an independent project 

## Method

Two models, cross-checked against each other:

1. **Static sizing model** (`sizing-scripts/crank_slider_sizing.py`) —
   quasi-static, angle-by-angle torque calculation from crank-slider
   kinematics and an idealised H2 indicator-diagram pressure cycle.
2. **Transient digital twin** (`models/digital_twin_transient.py`) —
   the same geometry and gas model, but integrated forward in time as a
   coupled ODE across three explicit domains: electric drive (torque
   feedforward + PI speed trim + first-order response lag), mechanical
   drivetrain (crank-slider + angle-dependent effective inertia), and
   gas thermofluidics (the indicator diagram, with valve transitions
   softened to represent finite opening/closing time).

A motor-bandwidth sweep (`models/motor_bandwidth_sweep.py`) then swept
the electric drive's response time to find the actual speed requirement
needed to hit the design target.

## Findings

1. **A passive flywheel is not viable at 1-30 RPM.** Required flywheel
   inertia to hold Cs=1.5% speed fluctuation scales as roughly 1/RPM²,
   from ~0.18 kg·m² at 2000 RPM to a physically absurd ~7×10⁵ kg·m² at
   1 RPM. Reason: a flywheel's stored energy scales with ω², but the
   gas-pressure torque pulse it must absorb does not shrink with speed.
2. **Reciprocating inertia is negligible relative to gas-pressure
   torque across the whole 1-2000 RPM range**, for this bore/pressure
   combination — confirmed under both the baseline geometry and a
   deliberately heavier/longer-stroke test case. The flywheel-sizing
   conclusion above is governed entirely by gas pressure, not by
   piston mass.
3. **A directly-driven, torque-controlled motor is the design
   consequence of (1).** Peak required torque ~156 N·m, peak power
   ~170 W at 10 RPM (a representative point in the target range).
4. **That motor needs real bandwidth, and this is quantified, not
   assumed:** to hold the Cs=1.5% target at 10 RPM, the motor's torque
   response time constant must be ≲1.6 ms (~100 Hz equivalent
   bandwidth). A generic VFD/simple drive (tens of ms response) is not
   sufficient; this points to a dedicated servo drive (PMSM/BLDC-class).

## Known limitations (carried forward, not resolved in this phase)

- Ideal-gas H2 polytropic model, not a real equation of state (H2's
  compressibility factor exceeds 1 at high pressure, unlike most gases)
- PI controller gains are first-pass, not optimised — a tuned
  controller might hit the target with a slower (cheaper) motor
- Only tested at 10 RPM; 1 RPM and 30 RPM ends of the target range not
  yet checked
- Valve transition angle (3°) is an engineering estimate, not a
  measured spec
- No experimental validation anywhere in this phase

## Bottom line

The original brief's question has a clear, non-obvious answer: the
high-speed industry's flywheel-based approach cannot be scaled down to
1-30 RPM — but a direct-drive servo solution can, provided the drive
has a fast enough torque loop, and this project quantifies exactly how
fast ("~100 Hz, ~1.6 ms") rather than leaving it as a qualitative
"needs to be fast" statement. This is the deliverable this phase set
out to produce, and it's now closed. CAD sizing (Phase 3) picks up
from the peak-load numbers this phase generated.
