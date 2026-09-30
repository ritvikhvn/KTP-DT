"""
single_rotor.py - Single-rotor pyBEMT solver.

Faithful port of upstream pyBEMT single-rotor equations. Adapts them to the
local codebase API: input via geo/polars dicts, output via standard result dict,
optional radial arrays for plotting and debugging.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import optimize

from aerodynamic_model.atmosphere import RHO_SL
from aerodynamic_model.airfoil import interpolate_polar
from aerodynamic_model.pyBEMT.pybemt.airfoil import load_airfoil

_MU_AIR = 1.81e-5  # dynamic viscosity of air [Pa*s] -- matches _FluidAdapter default


def solve_single_rotor(
    geo: dict,
    polars: dict,
    rpm: float,
    V_climb: float = 0.0,
    rho: float = RHO_SL,
    return_radial: bool = False,
) -> dict:
    """Solve an isolated rotor using upstream pyBEMT-equivalent equations."""
    if rpm <= 0.0:
        result = {
            "T_N": 0.0,
            "Q_Nm": 0.0,
            "P_aero_W": 0.0,
            "CT": 0.0,
            "CQ": 0.0,
            "J": 0.0,
            "v_i": 0.0,
            "eta": 0.0,
        }
        if return_radial:
            n = len(geo["r_norm"])
            z = np.zeros(n)
            result.update(
                {
                    "r": z.copy(),
                    "dT_dr": z.copy(),
                    "dQ_dr": z.copy(),
                    "v_i_arr": z.copy(),
                    "a_arr": z.copy(),
                    "ap_arr": z.copy(),
                    "phi_arr_deg": z.copy(),
                    "alpha_arr_deg": z.copy(),
                    "F_arr": z.copy(),
                }
            )
        return result

    r_norm = np.asarray(geo["r_norm"], dtype=float)
    chord = np.asarray(geo["chord"], dtype=float)
    twist_deg = np.asarray(geo["twist_deg"], dtype=float)
    R = float(geo["R"])
    B = int(geo["B"])
    D = 2.0 * R
    n_rev = rpm / 60.0
    omega = rpm * 2.0 * np.pi / 60.0
    r = r_norm * R

    # NeuralFoil polars (aerodynamic_model.airfoil.get_airfoil_polars) drive the
    # aerodynamics whenever they're supplied -- Cl/Cd interpolated per-section at an
    # estimated local Reynolds number, so any airfoil name actually affects the
    # solve. `polars=None` (or an explicit geo["upstream_airfoil"] override) falls
    # back to pyBEMT's bundled 5-airfoil table for backward compatibility.
    shared_airfoil = None
    if polars is None or "upstream_airfoil" in geo:
        shared_airfoil = load_airfoil(_resolve_upstream_airfoil_name(geo, polars))

    if len(r) >= 2:
        dr_default = float(r[1] - r[0])
    else:
        dr_default = max(R, 1e-6)
    dr = np.full_like(r, dr_default, dtype=float)
    radius_hub = max(float(r[0] - 0.5 * dr_default), 1e-6)

    rotor = _RotorAdapter(blade_radius=R, radius_hub=radius_hub, n_blades=B)
    fluid = _FluidAdapter(rho=rho)
    sections = [
        _SectionAdapter(
            airfoil=shared_airfoil if shared_airfoil is not None else _PolarAirfoil(
                polars, max(rho * abs(omega) * float(r_i) * float(chord_i) / _MU_AIR, 1.0)
            ),
            radius=float(r_i),
            width=float(dr_i),
            pitch_rad=math.radians(float(pitch_i)),
            chord=float(chord_i),
            rotor=rotor,
            mode="rotor",
        )
        for r_i, dr_i, pitch_i, chord_i in zip(r, dr, twist_deg, chord)
    ]

    T = 0.0
    Q = 0.0
    dT_dr = np.zeros_like(r)
    dQ_dr = np.zeros_like(r)
    a_arr = np.zeros_like(r)
    ap_arr = np.zeros_like(r)
    F_arr = np.zeros_like(r)
    phi_arr = np.zeros_like(r)
    alpha_arr_deg = np.zeros_like(r)
    v_i_arr = np.zeros_like(r)

    for i, sec in enumerate(sections):
        with np.errstate(divide="ignore", invalid="ignore"):
            try:
                phi = optimize.bisect(sec.func, 0.01 * np.pi, 0.9 * np.pi, args=(V_climb, omega))
            except ValueError:
                phi = _brute_solve(sec, V_climb, omega)
            dT, dQ = sec.forces(phi, V_climb, omega, fluid)

        T += dT
        Q += dQ
        dT_dr[i] = dT / max(sec.width, 1e-12)
        dQ_dr[i] = dQ / max(sec.width, 1e-12)
        a_arr[i] = sec.a
        ap_arr[i] = sec.ap
        F_arr[i] = sec.F
        phi_arr[i] = phi
        alpha_arr_deg[i] = sec.AoA
        v_i_arr[i] = (1.0 + sec.a) * V_climb - V_climb

    P = Q * omega
    A_disk = np.pi * R**2
    CT = T / (rho * n_rev**2 * D**4) if n_rev > 0.0 else 0.0
    CQ = Q / (rho * n_rev**2 * D**5) if n_rev > 0.0 else 0.0
    CP = 2.0 * np.pi * CQ
    J = V_climb / (n_rev * D) if n_rev > 0.0 else 0.0
    eta = J * CT / CP if CP > 1e-9 and J > 0.0 else 0.0
    v_i_mean = math.sqrt(max(T, 0.0) / (2.0 * rho * A_disk) + (V_climb / 2.0) ** 2) - V_climb / 2.0

    result = {
        "T_N": float(T),
        "Q_Nm": float(Q),
        "P_aero_W": float(P),
        "CT": float(CT),
        "CQ": float(CQ),
        "J": float(J),
        "v_i": float(max(v_i_mean, 0.0)),
        "eta": float(eta),
    }
    if return_radial:
        result.update(
            {
                "r": r.copy(),
                "dT_dr": dT_dr.copy(),
                "dQ_dr": dQ_dr.copy(),
                "v_i_arr": v_i_arr.copy(),
                "a_arr": a_arr.copy(),
                "ap_arr": ap_arr.copy(),
                "phi_arr_deg": np.degrees(phi_arr),
                "alpha_arr_deg": alpha_arr_deg.copy(),
                "F_arr": F_arr.copy(),
            }
        )
    return result


def _resolve_upstream_airfoil_name(geo: dict, polars: dict | None) -> str:
    if "upstream_airfoil" in geo:
        return str(geo["upstream_airfoil"])
    airfoil_name = str((polars or {}).get("airfoil", "")).strip().lower()
    mapping = {
        "clark_y": "CLARKY",
        "clarky": "CLARKY",
        "goe_408": "GOE_408",
        "goe408": "GOE_408",
        "goe_450": "GOE_450",
        "goe450": "GOE_450",
        "naca_4412": "NACA_4412",
        "naca4412": "NACA_4412",
        "naca_63815": "NACA_63815",
        "naca63815": "NACA_63815",
    }
    return mapping.get(airfoil_name, "CLARKY")


def _brute_solve(sec: "_SectionAdapter", v_inflow: float, omega: float, n: int = 3600) -> float:
    phis = np.linspace(-0.9 * np.pi, 0.9 * np.pi, n)
    resid = np.zeros(n)
    for i, phi in enumerate(phis):
        val = sec.func(phi, v_inflow, omega)
        resid[i] = val if np.isfinite(val) else 1e30
    return float(phis[int(np.argmin(np.abs(resid)))])


class _PolarAirfoil:
    """Exposes pyBEMT's Airfoil.Cl(alpha)/Cd(alpha) interface (radians in,
    coefficient out) backed by NeuralFoil-computed polars, bilinearly
    interpolated at a fixed Reynolds number estimated for this blade section."""

    def __init__(self, polars: dict, Re: float):
        self._polars = polars
        self._Re = Re

    def _alpha_deg(self, alpha_rad: float) -> float:
        return math.degrees(math.atan2(math.sin(alpha_rad), math.cos(alpha_rad)))

    def Cl(self, alpha_rad: float) -> float:
        return interpolate_polar(self._polars, self._Re, self._alpha_deg(alpha_rad))[0]

    def Cd(self, alpha_rad: float) -> float:
        return interpolate_polar(self._polars, self._Re, self._alpha_deg(alpha_rad))[1]


class _RotorAdapter:
    def __init__(self, blade_radius: float, radius_hub: float, n_blades: int):
        self.n_blades = n_blades
        self.blade_radius = blade_radius
        self.radius_hub = radius_hub


class _FluidAdapter:
    def __init__(self, rho: float, mu: float = 1.81e-5):
        self.rho = rho
        self.mu = mu


class _SectionAdapter:
    def __init__(
        self,
        *,
        airfoil,
        radius: float,
        width: float,
        pitch_rad: float,
        chord: float,
        rotor: _RotorAdapter,
        mode: str,
    ):
        self.airfoil = airfoil
        self.radius = radius
        self.width = width
        self.pitch = pitch_rad
        self.chord = chord
        self.rotor = rotor
        self.C = -1 if mode == "turbine" else 1

        self.v = 0.0
        self.v_theta = 0.0
        self.v_rel = 0.0
        self.a = 0.0
        self.ap = 0.0
        self.Re = 0.0
        self.alpha = 0.0
        self.AoA = 0.0
        self.dT = 0.0
        self.dQ = 0.0
        self.F = 0.0
        self.Cl = 0.0
        self.Cd = 0.0
        self.sigma = self.rotor.n_blades * self.chord / (2.0 * np.pi * max(self.radius, 1e-9))

    def tip_loss(self, phi: float) -> float:
        def prandtl(dr: float, r: float, phi_value: float) -> float:
            f = self.rotor.n_blades * dr / (2.0 * r * math.sin(phi_value))
            if -f > 500:
                F = 1.0
            else:
                F = 2.0 * math.acos(min(1.0, math.exp(-f))) / np.pi
            return F

        if phi == 0.0:
            F = 1.0
        else:
            r = self.radius
            Ftip = prandtl(self.rotor.blade_radius - r, r, phi)
            Fhub = prandtl(r - self.rotor.radius_hub, r, phi)
            F = Ftip * Fhub
        self.F = F
        return F

    def airfoil_forces(self, phi: float) -> tuple[float, float]:
        alpha = self.C * (self.pitch - phi)
        Cl = self.airfoil.Cl(alpha)
        Cd = self.airfoil.Cd(alpha)
        CT = Cl * math.cos(phi) - self.C * Cd * math.sin(phi)
        CQ = Cl * math.sin(phi) + self.C * Cd * math.cos(phi)
        self.AoA = math.degrees(alpha)
        self.Cl = float(Cl)
        self.Cd = float(Cd)
        return CT, CQ

    def induction_factors(self, phi: float) -> tuple[float, float]:
        F = self.tip_loss(phi)
        CT, CQ = self.airfoil_forces(phi)
        denom_t = self.sigma * CT
        denom_q = self.sigma * CQ
        kappa  = 4.0 * F * math.sin(phi) ** 2              / denom_t if abs(denom_t) > 1e-12 else math.copysign(1e9, denom_t or 1.0)
        kappap = 4.0 * F * math.sin(phi) * math.cos(phi)   / denom_q if abs(denom_q) > 1e-12 else math.copysign(1e9, denom_q or 1.0)
        a  = 1.0 / (kappa  - self.C) if abs(kappa  - self.C) > 1e-12 else math.copysign(1e9, kappa  - self.C or 1.0)
        ap = 1.0 / (kappap + self.C) if abs(kappap + self.C) > 1e-12 else math.copysign(1e9, kappap + self.C or 1.0)
        return a, ap

    def func(self, phi: float, v_inf: float, omega: float) -> float:
        a, ap = self.induction_factors(phi)
        denom_a  = 1.0 + self.C * a
        denom_ap = omega * max(self.radius, 1e-9) * (1.0 - self.C * ap)
        if abs(denom_a) < 1e-12 or abs(denom_ap) < 1e-12:
            self.a = a; self.ap = ap
            return 1e9  # large residual — bisect skips this region
        resid = math.sin(phi) / denom_a - v_inf * math.cos(phi) / denom_ap
        self.a = a
        self.ap = ap
        return resid

    def forces(self, phi: float, v_inf: float, omega: float, fluid: _FluidAdapter) -> tuple[float, float]:
        a, ap = self.induction_factors(phi)
        CT, CQ = self.airfoil_forces(phi)
        v = (1.0 + self.C * a) * v_inf
        vp = (1.0 - self.C * ap) * omega * self.radius
        U = math.sqrt(v**2 + vp**2)
        self.v = v
        self.v_theta = vp
        self.v_rel = U
        self.a = a
        self.ap = ap
        self.Re = fluid.rho * U * self.chord / fluid.mu
        self.dT = self.sigma * np.pi * fluid.rho * U**2 * CT * self.radius * self.width
        self.dQ = self.sigma * np.pi * fluid.rho * U**2 * CQ * self.radius**2 * self.width
        return self.dT, self.dQ
