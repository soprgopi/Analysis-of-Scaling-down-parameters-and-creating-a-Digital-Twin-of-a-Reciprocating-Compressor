# Low-Speed Reciprocating Gas Compressor — Digital Twin

Self-directed Master project investigating whether a mechanical
crankshaft-drive proven on large-scale, high-speed (100-2000 RPM)
reciprocating gas compressors can be adapted to a small-scale, low-speed
(1-30 RPM) machine — originally scoped as an industry thesis with
Maximator GmbH, continued independently after funding was withdrawn.

## Repository structure

```
docs/               design basis, write-ups, references
sizing-scripts/      Python sizing calculations (Phase 2, step 1)
models/              multiphysical digital twin (OpenModelica/Simscape) - not yet started
results/             generated plots and CSV output
cad/                 crankshaft/flywheel CAD (Phase 3) - not yet started
```

## Status

- [x] Design basis defined (`docs/design-basis.md`) — gas, pressures, RPM range,
      benchmarked against Maximator's public DLE gas-booster datasheets
- [x] Crank-slider kinematics + turning-moment-diagram flywheel sizing
      (`sizing-scripts/crank_slider_sizing.py`)
- [x] Three-domain multiphysical digital twin (electric drive / mechanical
      drivetrain / gas thermofluidics) — CONCLUDED, see
      `docs/phase2-digital-twin-conclusion.md`
- [ ] Crankshaft/flywheel CAD (started: first-pass shaft sizing done,
      see `cad/shaft_sizing.py`; full NX model not yet built)

## Headline result so far

Required flywheel inertia to hold speed fluctuation within Cs = 1.5% rises
from ~0.18 kg·m² at 2000 RPM to a physically absurd ~7×10⁵ kg·m² at 1 RPM
— an inverse-square relationship with RPM (see
`results/flywheel_inertia_vs_rpm.png`).

**Why:** the flywheel's stored kinetic energy scales with ω², but the gas-
pressure torque pulse that the flywheel has to absorb is essentially
RPM-independent (quasi-static compression cycle). So as RPM drops, you need
proportionally more inertia to buffer the same energy pulse — and at
1-30 RPM this becomes physically unbuildable.

This is a first, direct answer to the brief's original question: **at this
speed range, a passive flywheel is the wrong tool.** The likely path
forward is a directly-driven, torque/speed-controlled electric motor
(no flywheel, or a much smaller one paired with active control) rather
than the flywheel-smoothed approach used on high-speed machines.

## Known limitations / next refinements

- Reciprocating-inertia torque is negligible across the whole studied
  range for the current geometry (small bore, high pressure) — the
  torque-vs-angle curves at 1/30/500/2000 RPM overlap almost exactly in
  `results/torque_vs_angle.png`. Worth revisiting with a lighter piston /
  longer stroke to check whether inertia ever becomes significant.
- Indicator diagram is idealised (no valve dynamics, no leakage, no
  friction) — fine for first-pass sizing, not for the final digital twin.
- All geometry and pressure figures are representative, not real
  Maximator specifications (see `docs/design-basis.md` §7).

## Requirements

```
numpy
matplotlib
scipy
```

## Phase 2 progress: transient digital twin (models/digital_twin_transient.py)

First coupled electric-drive + mechanical-drivetrain simulation, gas
thermofluidics as the load. Run at 10 RPM (representative point in the
1-30 RPM target range).

**Solid results:**
- Peak motor torque ~156 N·m -- matches the sizing script's static
  torque-vs-angle result almost exactly (~155 N·m), a good cross-check
- Gives a first motor sizing spec: ~160 W peak, small-motor class

**Known limitations, not yet resolved:**
- Motor uses perfect torque feedforward (exact knowledge of the load at
  every instant) -- this is an idealised upper bound, not a real
  prediction. 0% speed fluctuation is an artifact of that idealisation,
  not a real finding. Next: add motor bandwidth/lag dynamics so the
  model shows realistic (nonzero) speed ripple.
- Mean motor power came out slightly negative; a rough isothermal hand
  calc predicts ~+16 W. Sign-convention bug in the torque direction,
  not yet fixed -- don't trust the power numbers until this is resolved.
- Still uses the ideal-gas H2 polytropic model (see design-basis.md
  Sec. 7), not a real H2 equation of state.

## Bug fix: torque sign convention (found via a power-sign inconsistency)

`T_gas = F_gas * dx_dtheta` had the load-torque sign inverted -- it
implied the motor needed negative torque during the expensive compression
stroke and positive torque during the cheap suction stroke, backwards
from physical reality. Corrected to `T_gas = -F_gas * dx_dtheta`
(same fix applied to the inertia-torque term and the digital twin's
T_load function).

**What changed:** `T_mean` and mean motor power flipped from negative
to correctly positive (+16.24 W, matching an isothermal hand-calc of
~16 W). The torque-vs-angle curve now has the recognisable shape of a
real compressor's turning-moment diagram (sharp positive peak near end
of compression, small negative dip during suction) instead of the
mirror image.

**What did NOT change:** the flywheel-inertia-vs-RPM headline result is
numerically identical to before (713,237 kg*m^2 at 1 RPM, unchanged
across the full RPM sweep). This is because the flywheel sizing method
integrates the *energy above the mean torque*, and for any periodic
signal the area above the mean always equals the area below it -- so a
uniform sign flip doesn't change that quantity. The original feasibility
conclusion (passive flywheel unbuildable at 1-30 RPM) was correct even
while this bug was present; it just happened to not depend on the part
that was wrong.

## Motor bandwidth requirement (models/motor_bandwidth_sweep.py)

Question: even with softened valve transitions, a generic 20ms-response
motor gives ~20% steady-state speed fluctuation -- 13x the Cs=1.5%
target. How fast does the motor actually need to be?

**Answer:** the motor's torque response time constant needs to be
**~1.6 ms or faster** (~100 Hz torque-control bandwidth equivalent) to
hold the 1.5% target at 10 RPM. Fluctuation scales cleanly with tau on
a log-log plot (see results/motor_bandwidth_sweep.png) -- halving tau
roughly halves the fluctuation.

**What this means for motor selection:** generic VFDs/simple drives
(typically tens of ms response) are not fast enough. This points to a
dedicated servo drive (PMSM/BLDC with a fast current loop), which
typically achieves a few hundred Hz to low-kHz bandwidth -- feasible,
but a real hardware requirement to carry into the CAD/component
selection phase, not a detail to skip.
