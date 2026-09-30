"""Standalone DC motor equivalent-circuit model (no propeller/aero coupling)."""
from __future__ import annotations

import numpy as np


def solve_motor_equilibrium(
    Kv: float, Rm: float, I0: float, V_batt: float, throttle: float,
    load_torque_Nm: float = 0.0, eta_ESC: float = 1.0, k_iron: float = 0.0, T_winding_C: float = 20.0,
) -> dict:
    """Return motor operating point from electrical parameters and a fixed mechanical load torque."""
    throttle = float(np.clip(throttle, 0.0, 1.0))
    Kv_si = float(Kv) * 2.0 * np.pi / 60.0
    Rm_eff = float(Rm) * (1.0 + 0.004 * (float(T_winding_C) - 20.0))
    V_eff = float(np.clip(eta_ESC, 0.0, 1.0)) * throttle * float(V_batt)
    if V_eff <= 0.0 or Kv_si <= 0.0 or Rm_eff <= 0.0:
        return _zero_result() | {"Rm_eff": Rm_eff, "I0_eff_at_eq": float(I0)}
    omega = max((V_eff / Rm_eff - float(I0) - float(load_torque_Nm) * Kv_si) / (1.0 / (Kv_si * Rm_eff) + float(k_iron)), 0.0)
    i0_eq = float(I0) + float(k_iron) * omega
    current = max((V_eff - omega / Kv_si) / Rm_eff, i0_eq)
    torque = max((current - i0_eq) / Kv_si, 0.0)
    shaft = torque * omega
    electrical = V_eff * current
    return {"rpm": omega * 60.0 / (2.0 * np.pi), "omega_rad_s": omega, "current_A": current, "torque_Nm": torque, "V_motor": V_eff, "P_shaft_W": shaft, "P_elec_W": electrical, "eta_motor": shaft / electrical if electrical > 1e-9 else 0.0, "Rm_eff": Rm_eff, "I0_eff_at_eq": i0_eq}


def _zero_result() -> dict:
    return {"rpm": 0.0, "omega_rad_s": 0.0, "current_A": 0.0, "torque_Nm": 0.0, "V_motor": 0.0, "P_shaft_W": 0.0, "P_elec_W": 0.0, "eta_motor": 0.0, "Rm_eff": 0.0, "I0_eff_at_eq": 0.0}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="DC motor equivalent-circuit solver")
    parser.add_argument("--kv", type=float, default=920.0, help="motor Kv [RPM/V]")
    parser.add_argument("--rm", type=float, default=0.115, help="winding resistance [ohm]")
    parser.add_argument("--i0", type=float, default=0.8, help="no-load current [A]")
    parser.add_argument("--v-batt", type=float, default=14.8, help="battery voltage [V]")
    parser.add_argument("--throttle", type=float, default=0.7, help="throttle 0-1")
    parser.add_argument("--load-torque-nm", type=float, default=0.05, help="external load torque [Nm]")
    parser.add_argument("--eta-esc", type=float, default=1.0, help="ESC efficiency 0-1")
    parser.add_argument("--k-iron", type=float, default=0.0, help="iron/eddy loss coefficient")
    parser.add_argument("--t-winding-c", type=float, default=20.0, help="winding temperature [C]")
    args = parser.parse_args()

    result = solve_motor_equilibrium(
        args.kv, args.rm, args.i0, args.v_batt, args.throttle,
        load_torque_Nm=args.load_torque_nm, eta_ESC=args.eta_esc, k_iron=args.k_iron, T_winding_C=args.t_winding_c,
    )
    assert np.isfinite(result["rpm"]) and result["rpm"] > 0
    assert result["current_A"] >= 0 and 0 <= result["eta_motor"] <= 1
    print(result)
