"""
Transient digital twin: electric drive + mechanical drivetrain + gas
thermofluidic load, for the low-speed H2 crank-slider compressor.

This is the actual digital twin (Phase 2), distinct from
sizing-scripts/crank_slider_sizing.py, which only did a quasi-static,
angle-by-angle calculation. Here the system is integrated forward in
TIME as a coupled ODE:

    theta_dot  = omega
    omega_dot  = (T_motor_actual(t) - T_load(theta)) / I_eff(theta)
    T_motor_actual approaches its commanded value with a first-order
    lag (TAU_MOTOR) -- see Section 5.

Three domains, explicitly separated so each can be swapped out later:
    1. ELECTRIC DRIVE   -- torque feedforward + PI speed trim, with a
                            first-order lag representing real motor/
                            drive response time (not instantaneous).
    2. MECHANICAL DRIVETRAIN -- crank-slider kinematics (reused from the
                            sizing script) plus an angle-dependent
                            effective inertia (rotor/shaft + the piston's
                            own contribution, which varies with crank
                            angle).
    3. GAS THERMOFLUIDICS -- the same idealised H2 indicator-diagram
                            pressure model as the sizing script, with
                            suction/discharge transitions softened by a
                            Gaussian filter to represent finite valve
                            opening/closing (not a measured valve spec).
                            Still an ideal-gas polytropic model, not a
                            real H2 EOS -- flagged for future upgrade.

Run this after sizing-scripts/crank_slider_sizing.py; geometry and
pressure parameters are kept identical to that script's design-basis
values so results are directly comparable.

Run directly (`python digital_twin_transient.py`) for a single run at
TAU_MOTOR below. For the motor-bandwidth requirement sweep, see
models/motor_bandwidth_sweep.py, which imports run() from this file.
"""

import os
import numpy as np
from scipy.integrate import solve_ivp
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "..", "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# ---------------------------------------------------------------------
# 1. Geometry / gas parameters (identical to design-basis.md / the
#    sizing script's ORIGINAL values -- r=0.015, m_recip=0.15)
# ---------------------------------------------------------------------
D = 0.030
r = 0.015
l = 0.060
clearance_frac = 0.05
p1 = 20e5
p2 = 200e5
n_poly = 1.3          # ideal-gas polytropic approx for H2 -- see note above
m_recip = 0.15

A = np.pi / 4 * D**2
Vs = A * (2 * r)
Vc = clearance_frac * Vs
lam = r / l

# ---------------------------------------------------------------------
# 2. Crank-slider kinematics as callable functions of theta
# ---------------------------------------------------------------------
def piston_x(theta):
    return r * (1 - np.cos(theta)) + r * (lam / 4) * (1 - np.cos(2 * theta))

def dx_dtheta(theta):
    return r * np.sin(theta) + r * (lam / 2) * np.sin(2 * theta)

# ---------------------------------------------------------------------
# 3. Gas pressure as a function of theta (precomputed grid, interpolated)
# ---------------------------------------------------------------------
_theta_grid = np.linspace(0, 2 * np.pi, 7200, endpoint=False)
_V_grid = Vc + A * piston_x(_theta_grid)
_V_at_p1 = Vc * (p2 / p1) ** (1 / n_poly)

_p_grid = np.empty_like(_V_grid)
for i, Vi in enumerate(_V_grid):
    _p_grid[i] = p2 * (Vc / Vi) ** n_poly if Vi <= _V_at_p1 else p1

_bdc_idx = np.argmax(_V_grid)
for i in range(_bdc_idx, len(_theta_grid)):
    Vi = _V_grid[i]
    p_comp = p1 * (_V_grid[_bdc_idx] / Vi) ** n_poly if Vi < _V_grid[_bdc_idx] else p1
    if p_comp >= p2:
        _p_grid[i] = p2
    elif Vi < _V_grid[_bdc_idx]:
        _p_grid[i] = p_comp
_p_grid = np.clip(_p_grid, p1, p2)

# Soften suction/discharge transitions (finite valve opening/closing,
# ~3 deg of crank angle -- representative first-pass estimate, not a
# measured valve spec)
_valve_transition_deg = 3.0
_sigma_points = _valve_transition_deg / 360.0 * len(_theta_grid)
_p_grid = gaussian_filter1d(_p_grid, sigma=_sigma_points, mode="wrap")
_p_grid = np.clip(_p_grid, p1, p2)

def gas_pressure(theta):
    theta_wrapped = np.mod(theta, 2 * np.pi)
    return np.interp(theta_wrapped, _theta_grid, _p_grid)

def T_load(theta):
    """Torque the motor must supply to overcome the gas at angle theta.
    Sign convention: F_gas always pushes the piston away from the head;
    during compression the motor must push against that, so required
    torque = -F_gas * dx_dtheta."""
    return -gas_pressure(theta) * A * dx_dtheta(theta)

# ---------------------------------------------------------------------
# 4. Effective inertia at the crankshaft, angle-dependent
# ---------------------------------------------------------------------
I_rotor = 0.01  # kg*m^2, small motor rotor + crankshaft -- NO large flywheel,
                 # per the sizing-script conclusion that passive flywheels
                 # are not viable at 1-30 RPM

def I_eff(theta):
    return I_rotor + m_recip * dx_dtheta(theta) ** 2

# ---------------------------------------------------------------------
# 5. Electric drive: torque feedforward + PI speed trim + first-order lag
# ---------------------------------------------------------------------
RPM_TARGET = 10  # representative point inside the 1-30 RPM target range
OMEGA_REF = 2 * np.pi * RPM_TARGET / 60.0

KP = 40.0    # N*m per rad/s of speed error -- first-pass tuning, not optimised
KI = 200.0   # N*m per rad of accumulated speed error
T_MOTOR_MAX = 250.0  # N*m -- must exceed peak gas-load torque (~148-156 N.m)

TAU_MOTOR = 0.02  # s, first-order motor torque response lag (baseline
                    # "generic small servo/BLDC drive" estimate). See
                    # motor_bandwidth_sweep.py for what value is actually
                    # required to meet the Cs=1.5% speed-fluctuation target.

def motor_torque_command(theta, omega, integral_error):
    T_feedforward = T_load(theta)
    error = OMEGA_REF - omega
    T_trim = KP * error + KI * integral_error
    return np.clip(T_feedforward + T_trim, -T_MOTOR_MAX, T_MOTOR_MAX)

# ---------------------------------------------------------------------
# 6. Coupled ODE + single-run driver, callable with any tau_motor
# ---------------------------------------------------------------------
def _rhs(t, state, tau_motor):
    theta, omega, integral_error, T_motor_actual = state
    T_cmd = motor_torque_command(theta, omega, integral_error)
    T_l = T_load(theta)
    domega = (T_motor_actual - T_l) / I_eff(theta)
    dintegral = OMEGA_REF - omega
    dT_motor = (T_cmd - T_motor_actual) / tau_motor
    return [omega, domega, dintegral, dT_motor]

def run(tau_motor=TAU_MOTOR, rpm_target=RPM_TARGET, n_revs=5, make_plot=True):
    """Run the coupled digital twin at the given motor lag time constant.
    Returns a dict of headline results. Used both for a single diagnostic
    run (see __main__ below) and by motor_bandwidth_sweep.py."""
    omega_ref = 2 * np.pi * rpm_target / 60.0
    t_end = n_revs * 60.0 / rpm_target
    t_eval = np.linspace(0, t_end, 4000)
    T_motor_actual_0 = T_load(0.0)

    sol = solve_ivp(
        _rhs, [0, t_end], [0.0, omega_ref, 0.0, T_motor_actual_0],
        args=(tau_motor,), t_eval=t_eval, method="RK45", rtol=1e-8, atol=1e-10,
    )

    theta_t, omega_t, integral_t, T_motor_t = sol.y
    T_cmd_t = np.clip(
        T_load(theta_t) + KP * (omega_ref - omega_t) + KI * integral_t,
        -T_MOTOR_MAX, T_MOTOR_MAX,
    )
    T_load_t = T_load(theta_t)
    power_t = T_motor_t * omega_t

    rpm_t = omega_t * 60 / (2 * np.pi)
    full_fluctuation_pct = (rpm_t.max() - rpm_t.min()) / rpm_target * 100
    last_rev_mask = sol.t >= (t_end - 60.0 / rpm_target)
    steady_rpm = rpm_t[last_rev_mask]
    steady_fluctuation_pct = (steady_rpm.max() - steady_rpm.min()) / rpm_target * 100

    results = {
        "tau_motor": tau_motor,
        "rpm_target": rpm_target,
        "full_fluctuation_pct": full_fluctuation_pct,
        "steady_fluctuation_pct": steady_fluctuation_pct,
        "peak_torque_Nm": np.max(np.abs(T_motor_t)),
        "peak_power_W": np.max(np.abs(power_t)),
        "mean_power_W": np.mean(power_t),
    }

    if make_plot:
        fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
        axes[0].plot(sol.t, rpm_t)
        axes[0].axhline(rpm_target, color="k", linewidth=0.5, linestyle="--")
        axes[0].set_ylabel("Shaft speed [RPM]")
        axes[0].set_title(f"Transient digital twin at {rpm_target} RPM target "
                           f"(no flywheel, tau_motor={tau_motor*1000:.1f} ms)")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(sol.t, T_cmd_t, label="Motor torque (commanded)", alpha=0.6, linestyle="--")
        axes[1].plot(sol.t, T_motor_t, label="Motor torque (actual, lagged)")
        axes[1].plot(sol.t, T_load_t, label="Gas load torque", alpha=0.7)
        axes[1].set_ylabel("Torque [N·m]")
        axes[1].legend(fontsize=8)
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(sol.t, power_t)
        axes[2].set_ylabel("Motor power [W]")
        axes[2].set_xlabel("Time [s]")
        axes[2].grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(os.path.join(RESULTS_DIR, "digital_twin_transient.png"), dpi=150)
        plt.close(fig)

    return results

# ---------------------------------------------------------------------
# 7. Single diagnostic run
# ---------------------------------------------------------------------
if __name__ == "__main__":
    r = run(tau_motor=TAU_MOTOR)
    print(f"Target speed: {r['rpm_target']} RPM, motor torque lag time constant: {r['tau_motor']*1000:.2f} ms")
    print(f"Speed fluctuation, full run incl. startup transient (peak-to-peak): {r['full_fluctuation_pct']:.3f}% of target")
    print(f"Speed fluctuation, steady-state only (last revolution): {r['steady_fluctuation_pct']:.3f}% of target")
    print(f"Peak motor torque required: {r['peak_torque_Nm']:.2f} N·m (limit set at {T_MOTOR_MAX} N·m)")
    print(f"Peak motor power required: {r['peak_power_W']:.2f} W")
    print(f"Mean motor power: {r['mean_power_W']:.2f} W")
