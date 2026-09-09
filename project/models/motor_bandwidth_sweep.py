"""
Motor torque-bandwidth requirement sweep.

Answers a concrete question raised by digital_twin_transient.py: with a
generic 20ms motor response, steady-state speed fluctuation is ~20% of
target -- far above the Cs=1.5% design target from docs/design-basis.md.
Softening the valve transitions only partly helped (44% -> 20%). This
script sweeps the motor's torque response time constant (TAU_MOTOR) to
find how fast a real drive actually needs to be, turning that into a
concrete motor/drive selection spec rather than a vague "needs to be
faster."
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import csv
from digital_twin_transient import run, RPM_TARGET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "..", "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

CS_TARGET_PCT = 1.5  # design target from docs/design-basis.md

TAU_SWEEP_MS = [20, 10, 5, 2, 1, 0.5, 0.2]

results = []
for tau_ms in TAU_SWEEP_MS:
    r = run(tau_motor=tau_ms / 1000.0, make_plot=False)
    results.append(r)
    print(f"tau = {tau_ms:>5.1f} ms -> steady-state fluctuation = "
          f"{r['steady_fluctuation_pct']:>6.3f}% "
          f"(peak torque {r['peak_torque_Nm']:.1f} N.m)")

# Find where the sweep crosses the target (log-log linear interpolation)
taus = np.array([r["tau_motor"] for r in results]) * 1000  # ms
flucts = np.array([r["steady_fluctuation_pct"] for r in results])
log_tau = np.log10(taus)
log_fluct = np.log10(flucts)
# results are ordered from slow (high fluctuation) to fast (low fluctuation)
required_log_tau = np.interp(np.log10(CS_TARGET_PCT), log_fluct[::-1], log_tau[::-1])
required_tau_ms = 10 ** required_log_tau

fig, ax = plt.subplots(figsize=(7, 5))
ax.loglog(taus, flucts, marker="o")
ax.axhline(CS_TARGET_PCT, color="r", linestyle="--",
           label=f"Design target Cs = {CS_TARGET_PCT}%")
ax.axvline(required_tau_ms, color="g", linestyle=":",
           label=f"Required tau ~= {required_tau_ms:.2f} ms")
ax.set_xlabel("Motor torque response time constant, tau [ms]")
ax.set_ylabel("Steady-state speed fluctuation [%]")
ax.set_title(f"Motor bandwidth requirement at {RPM_TARGET} RPM")
ax.legend()
ax.grid(True, which="both", alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS_DIR, "motor_bandwidth_sweep.png"), dpi=150)

with open(os.path.join(RESULTS_DIR, "motor_bandwidth_sweep.csv"), "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
    writer.writeheader()
    writer.writerows(results)

bandwidth_hz = 1 / (2 * np.pi * required_tau_ms / 1000.0)
print()
print(f"Required motor torque-loop time constant to meet Cs={CS_TARGET_PCT}%: "
      f"~{required_tau_ms:.2f} ms")
print(f"Equivalent torque-control bandwidth: ~{bandwidth_hz:.0f} Hz")
print("For reference: generic VFDs/simple drives are typically tens of ms "
      "(too slow); modern industrial servo drives (PMSM/BLDC with a fast "
      "current loop) typically achieve a few hundred Hz to low kHz "
      "(sufficient) -- this points to a dedicated servo drive, not a "
      "generic motor/VFD combination.")
