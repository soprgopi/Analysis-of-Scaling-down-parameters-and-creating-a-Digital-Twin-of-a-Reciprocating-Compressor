# Design Basis — Low-Speed Reciprocating Gas Compressor Digital Twin

**Status:** Draft v0.2 (gas corrected: N2 -> H2)
**Author:** Prayag
**Date:** 2026-09-08

## 1. Context and motivation

Maximator's established product line is the air-driven, double-acting piston
**gas booster** (pressure intensifier): a pneumatic drive piston forces a
smaller high-pressure piston to compress process gas, with no crankshaft or
rotating drivetrain involved. Ratios across their DLE family range from
1:2 up to 1:150, at inlet pressures from a few bar up to several hundred
bar, with flows in the 90-130 l/min range at typical pneumatic cycle rates.

The idea explored in the original (now unfunded) thesis brief was different:
replace the pneumatic air-drive with an **electric motor driving a
mechanical crankshaft**, borrowed from the design practice of large-scale,
high-speed (100-2000 RPM) industrial reciprocating compressors — but applied
to a small-scale machine running at only **1-30 RPM**. This is a genuinely
open question: crankshaft/flywheel dynamics, torque ripple, and belt-vs-gear
transmission choices that are well-understood at 100+ RPM behave very
differently in this low-speed regime.

This project builds a physics-based digital twin to answer that question
directly, using representative (not confidential) specifications benchmarked
against Maximator's public datasheets.

## 2. Working gas

**Selected: Hydrogen (H2)** (corrected from an earlier nitrogen draft)

- Matches Maximator's own product range — their gas boosters explicitly
  list hydrogen alongside nitrogen, argon, and helium, compressed up to
  2,100 bar
- Brings real constraints nitrogen does not, all carried forward into
  later phases:
  - **Explosion protection:** Maximator's actual hydrogen-capable units
    are rated for hazardous areas (Zone 1, device group II, category 2G).
    The electric-drive domain of the digital twin needs an ATEX-rated
    motor/enclosure assumption, not an arbitrary off-the-shelf motor.
  - **Material compatibility:** hydrogen embrittlement rules out certain
    high-strength steels for the crankshaft/cylinder — CAD phase (Phase 3)
    should default to an H2-compatible alloy (e.g. 316L stainless, matching
    Maximator's own gas-section material) rather than a generic steel.
  - **Sealing:** H2's small molecule size drives high permeation through
    elastomers — Maximator's own datasheets specify PTFE seals for this
    reason; carry the same assumption forward.
  - **Real-gas behaviour:** unlike most gases, hydrogen's compressibility
    factor Z exceeds 1 at high pressure (it deviates from ideal-gas
    behaviour in the opposite direction to nitrogen or CO2). The polytropic
    indicator-diagram model in the sizing script does not capture this —
    it's a known simplification to fix when the thermofluidic domain of
    the digital twin is built (Phase 2), ideally via CoolProp's real
    hydrogen equation of state rather than an ideal-gas polytropic
    assumption.

## 3. Pressure levels

**Inlet: 20 bar | Outlet: 200 bar | Pressure ratio: 1:10, single-stage**

Chosen as a representative mid-range point inside Maximator's actual DLE
ratio spread (1:2 to 1:150), high enough that real-gas effects matter for
the thermofluidic model (worth modeling properly), but not so high that a
single-stage design becomes unrealistic.

## 4. Mass flow / displacement

**Target: ~5-10 l/min (free air delivery equivalent), displacement volume
sized in Phase 2 sizing script, not fixed here**

Deliberately left as an output of the sizing calculation rather than an
input: at 1-30 RPM the achievable flow is fundamentally different from the
90-130 l/min seen on Maximator's pneumatically-cycled boosters, and the gap
between the two is itself part of the feasibility story. The sizing script
(Step 4 of the project plan) will calculate flow as a function of bore,
stroke, and the RPM sweep.

## 5. Speed / RPM range

**Target operating range: 1-30 RPM**
**Baseline for comparison: 100-2000 RPM (established high-speed recip
practice)**

This is the central independent variable for the whole feasibility study.

## 6. Machine footprint / power (order-of-magnitude targets)

- Portable/bench-scale unit, footprint comparable to Maximator's smaller
  DLE units (roughly 0.2-0.8 m footprint, 20-25 kg class)
- Electric drive power: low, single-digit kW range — to be confirmed once
  the sizing script gives torque/power vs. RPM

## 7. Assumptions and limitations (stated explicitly)

- All figures above are representative, literature-benchmarked values —
  **not** real Maximator specifications, since no company data is available
- No physical prototype or test rig is in scope; validation will be against
  literature/hand-calculation references, not measured data
- Model starts single-stage; multi-stage is a possible extension if the
  1:10 ratio proves unrealistic for the flywheel/torque story
- The sizing-script polytropic model (n = 1.3) treats H2 as an ideal gas;
  this is acceptable for a first-pass flywheel/torque sizing exercise
  (torque there depends on p·A, not on gas identity) but should be
  replaced with a real-gas EOS before the thermofluidic digital-twin
  domain reports mass flow or discharge temperature

## 8. Open questions carried into Phase 2 (sizing script)

1. How does required flywheel inertia scale as RPM drops from 100-2000 down
   to 1-30?
2. At 1-30 RPM, does the flywheel become a hindrance (excess mass, no
   smoothing benefit) rather than a help?
3. Is a belt drive still torque-viable at this speed, or does a geared
   motor become mandatory?
