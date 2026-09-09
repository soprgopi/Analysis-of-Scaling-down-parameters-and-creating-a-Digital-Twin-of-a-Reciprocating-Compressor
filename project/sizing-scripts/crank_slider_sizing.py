"""
Crank-slider kinematics + flywheel inertia sizing for a low-speed
reciprocating gas compressor.

Answers the core feasibility question from docs/design-basis.md:
    How does the required flywheel inertia change as RPM drops from the
    established high-speed recip range (100-2000 RPM) down to the
    Maximator target range (1-30 RPM), and does the flywheel become a
    hindrance rather than a help at low speed?

Method: classic turning-moment-diagram approach (machine design textbook
method, e.g. Shigley / Norton), extended to separate the two torque
contributors explicitly:
    1. Gas-pressure torque   -- from an idealised indicator diagram,
                                 independent of RPM (quasi-static assumption)
    2. Reciprocating-inertia torque -- from d'Alembert force of the piston
                                 assembly, scales with omega^2

This separation is the point of the script: at low RPM the inertia term
vanishes and only the gas-pressure fluctuation remains to size the
flywheel against, which behaves very differently from the high-RPM case.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import csv

# Resolve results/ relative to this script's own location, not the
# current working directory -- makes the script work the same whether
# you run it via VS Code's Run button, a terminal, or by double-click.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "..", "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# ---------------------------------------------------------------------
# 1. Design basis parameters (see docs/design-basis.md)
# ---------------------------------------------------------------------
D = 0.030            # bore, m
r = 0.015             # crank radius, m (stroke = 2r = 30 mm)
l = 0.060             # connecting-rod length, m (l/r = 4)
clearance_frac = 0.05  # clearance volume as fraction of swept volume
p1 = 20e5             # suction pressure, Pa (20 bar)
p2 = 200e5            # discharge pressure, Pa (200 bar)
# Working gas: hydrogen (H2), matching Maximator's actual product range.
# n_poly = 1.3 is an ideal-diatomic-gas approximation, same as would be used
# for nitrogen. This is a KNOWN SIMPLIFICATION: unlike most gases, H2's
# compressibility factor Z exceeds 1 at high pressure, so it deviates from
# ideal-gas behaviour in the opposite direction to nitrogen. Fine for this
# torque/flywheel sizing pass (torque here depends on p*A, not gas
# identity) but must be replaced with a real-gas EOS (e.g. CoolProp's H2
# model) before this script's mass-flow numbers are trusted.
n_poly = 1.3
m_recip = 0.15         # reciprocating mass (piston + wrist pin + part of rod), kg
Cs = 0.015             # target coefficient of speed fluctuation (1.5%)

A = np.pi / 4 * D**2         # piston area, m^2
Vs = A * (2 * r)             # swept volume, m^3
Vc = clearance_frac * Vs     # clearance volume, m^3

RPM_LIST = [1, 2, 5, 10, 20, 30, 50, 100, 200, 500, 1000, 1500, 2000]
N_THETA = 3600  # angular resolution, points per revolution

# ---------------------------------------------------------------------
# 2. Crank-slider kinematics
# ---------------------------------------------------------------------
theta = np.linspace(0, 2 * np.pi, N_THETA, endpoint=False)
lam = r / l

# Piston displacement from TDC (standard slider-crank expression)
x = r * (1 - np.cos(theta)) + r * (lam / 4) * (1 - np.cos(2 * theta))

# dx/dtheta and d2x/dtheta2 via analytic differentiation of the same
# approximation (consistent with x, avoids numerical-derivative noise)
dx_dtheta = r * np.sin(theta) + r * (lam / 2) * np.sin(2 * theta)
d2x_dtheta2 = r * np.cos(theta) + r * lam * np.cos(2 * theta)

# Cylinder volume as a function of crank angle
V = Vc + A * x

# ---------------------------------------------------------------------
# 3. Idealised indicator diagram (real cycle with clearance re-expansion)
# ---------------------------------------------------------------------
# TDC is theta = 0. Cycle order as theta increases from 0 to 2*pi:
#   a) Re-expansion of clearance gas from p2 down to p1 (polytropic)
#   b) Suction at constant p1 until BDC (theta = pi) then back some way
#   c) Compression of trapped gas from p1 up to p2 (polytropic)
#   d) Discharge at constant p2 back to TDC
V_at_p1_from_p2 = Vc * (p2 / p1) ** (1 / n_poly)  # volume where re-expansion hits p1

p = np.empty_like(V)
for i, Vi in enumerate(V):
    if Vi <= V_at_p1_from_p2:
        # still re-expanding from clearance gas
        p[i] = p2 * (Vc / Vi) ** n_poly
    else:
        p[i] = p1

# second pass: compression side (piston moving back toward TDC)
# find index of BDC to split the array into expansion+suction half vs
# compression+discharge half
bdc_idx = np.argmax(V)
for i in range(bdc_idx, N_THETA):
    Vi = V[i]
    p_comp = p1 * (V[bdc_idx] / Vi) ** n_poly if Vi < V[bdc_idx] else p1
    if p_comp >= p2:
        p[i] = p2
    else:
        p[i] = max(p[i], p_comp) if Vi >= V_at_p1_from_p2 else p[i]
        if Vi < V[bdc_idx]:
            p[i] = p_comp

# clip for sanity
p = np.clip(p, p1, p2)

# ---------------------------------------------------------------------
# 4. Gas-pressure torque -- the torque the MOTOR must supply to the
#    crankshaft to overcome the gas (RPM-independent, quasi-static
#    assumption).
#
#    SIGN CONVENTION (fixed 2026 -- was inverted in the first version
#    of this script): F_gas = p*A always pushes the piston AWAY from
#    the cylinder head. During compression (piston moving toward TDC,
#    dx_dtheta < 0) the motor must push AGAINST that force, so the
#    required torque must be positive there. That means:
#        T_load = -F_gas * dx_dtheta
#    (not +F_gas * dx_dtheta, which gives negative torque during the
#    expensive compression stroke and positive during the cheap
#    suction stroke -- backwards).
# ---------------------------------------------------------------------
F_gas = p * A                       # N, gas force on piston (always pushes away from head)
T_gas = -F_gas * dx_dtheta          # N*m per radian, torque required FROM the motor

results = []
fig1, ax1 = plt.subplots(figsize=(7, 5))

for rpm in RPM_LIST:
    omega = 2 * np.pi * rpm / 60.0  # rad/s

    # Reciprocating inertia force (d'Alembert): F = -m * a, a = omega^2 * d2x/dtheta2
    # Same sign convention as T_gas above: required torque contribution = -F * dx_dtheta
    F_inertia = -m_recip * omega**2 * d2x_dtheta2
    T_inertia = -F_inertia * dx_dtheta

    T_total = T_gas + T_inertia
    T_mean = np.mean(T_total)

    # Turning-moment-diagram method: integrate positive excursions above mean
    deviation = T_total - T_mean
    positive = np.clip(deviation, 0, None)
    dtheta = theta[1] - theta[0]
    delta_E = np.sum(positive) * dtheta  # J (torque * angle = energy)

    I_flywheel = delta_E / (Cs * omega**2) if omega > 0 else np.nan

    results.append({
        "rpm": rpm,
        "omega_rad_s": omega,
        "T_mean_Nm": T_mean,
        "delta_E_J": delta_E,
        "I_flywheel_kgm2": I_flywheel,
        "flow_l_min": Vs * rpm * 1000,  # 1 discharge/rev, ideal volumetric eff.
    })

    if rpm in (1, 30, 500, 2000):
        ax1.plot(np.degrees(theta), T_total, label=f"{rpm} RPM")

ax1.axhline(0, color="k", linewidth=0.5)
ax1.set_xlabel("Crank angle [deg]")
ax1.set_ylabel("Total torque [N·m]")
ax1.set_title("Torque vs crank angle at selected speeds")
ax1.legend()
ax1.grid(True, alpha=0.3)
fig1.tight_layout()
fig1.savefig(os.path.join(RESULTS_DIR, "torque_vs_angle.png"), dpi=150)

# ---------------------------------------------------------------------
# 5. Required flywheel inertia vs RPM -- the headline result
# ---------------------------------------------------------------------
rpms = [row["rpm"] for row in results]
I_vals = [row["I_flywheel_kgm2"] for row in results]

fig2, ax2 = plt.subplots(figsize=(7, 5))
ax2.loglog(rpms, I_vals, marker="o")
ax2.axvspan(1, 30, color="orange", alpha=0.15, label="Maximator target range (1-30 RPM)")
ax2.axvspan(100, 2000, color="blue", alpha=0.1, label="Established high-speed recip range")
ax2.set_xlabel("RPM")
ax2.set_ylabel("Required flywheel inertia I [kg·m²]")
ax2.set_title("Required flywheel inertia vs operating speed")
ax2.legend()
ax2.grid(True, which="both", alpha=0.3)
fig2.tight_layout()
fig2.savefig(os.path.join(RESULTS_DIR, "flywheel_inertia_vs_rpm.png"), dpi=150)

# ---------------------------------------------------------------------
# 6. Save numeric results
# ---------------------------------------------------------------------
with open(os.path.join(RESULTS_DIR, "sizing_results.csv"), "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
    writer.writeheader()
    writer.writerows(results)

print(f"Swept volume Vs = {Vs*1e6:.2f} cm^3, clearance Vc = {Vc*1e6:.2f} cm^3")
print(f"{'RPM':>6} {'T_mean [N.m]':>14} {'I_flywheel [kg.m2]':>20} {'Flow [l/min]':>14}")
for row in results:
    print(f"{row['rpm']:>6} {row['T_mean_Nm']:>14.4f} {row['I_flywheel_kgm2']:>20.4f} {row['flow_l_min']:>14.3f}")
