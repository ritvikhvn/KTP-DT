"""
single_rotor.py — Single-rotor operating point analysis.

For each combination of inputs computes T, Q, P_aero, CT, CQ, J, eta and the
radial blade-loading distributions dT/dr, dQ/dr, and per-station induced
velocity v_i(r/R).

List inputs are expanded into cases and overlaid on shared axes.
SWEEP_VAR + SWEEP_VALUES trigger a parametric sweep of any single scalar key.

CONFIG BLOCK — edit constants below for standalone CLI use.
GUI use: call run(config) -> dict.
"""

from __future__ import annotations

import itertools
import argparse
import warnings
from pathlib import Path
import math as _math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# -- Repo-root resolution -----------------------------------------------------
_REPO_ROOT = Path(__file__).parent.parent
_APC_DIR   = _REPO_ROOT / "data" / "apc_geo" / "PE0-FILES_WEB"

# -- CONFIG -------------------------------------------------------------------
# Each scalar field also accepts a list — all combinations are run and
# overlaid on shared plots.
# SWEEP_VAR + SWEEP_VALUES provide a named parametric sweep of a single key
# (overrides that key's value in CONFIG for each sweep point).
CONFIG = {
    "D_IN":    10.0,          # propeller diameter [in]    -- float or list
    "P_IN":    4.5,           # propeller pitch [in]       -- float or list
    "B":       2,             # blade count
    "RPM":     6000.0,        # rotational speed [RPM]     -- float or list
    "V_CLIMB": 0.0,           # axial climb velocity [m/s] -- float or list
    "AIRFOIL": "clark_y",     # airfoil name               -- str or list
    # Parametric sweep (optional) -- overrides the named key across SWEEP_VALUES
    "SWEEP_VAR":    None,     # e.g. "RPM", "V_CLIMB", "D_IN"
    "SWEEP_VALUES": None,     # e.g. [4000, 5000, 6000, 7000, 8000]

    # Geometry source
    "DESIGN_RPM":  6000,
    "DESIGN_T_N":  5.0,
    # Output control
    "MAKE_PLOTS":    True,
    "PRINT_SUMMARY": True,
}
# -----------------------------------------------------------------------------

_IN = 0.0254  # inches -> metres


def _to_list(v):
    return v if isinstance(v, list) else [v]


def _load_geo_polars(D_m, P_m, B, airfoil, config):
    """Load geometry and polars for one (D, P, airfoil) combination."""
    from aerodynamic_model.geometry import get_blade_geometry
    from aerodynamic_model.airfoil import get_airfoil_polars
    geo = get_blade_geometry(D_m, P_m, B, design_rpm=config["DESIGN_RPM"], design_thrust_N=config["DESIGN_T_N"], apc_data_dir=str(_APC_DIR))
    polars = get_airfoil_polars(airfoil)
    src = geo["source"]
    return geo, polars, src


def _build_combos(config: dict):
    """
    Expand config into a flat list of scalar dicts, one per case.

    SWEEP_VAR / SWEEP_VALUES inserts a sweep along a single axis; all other
    list fields are also expanded via itertools.product.
    """
    cfg = dict(config)

    # Apply sweep override
    sv = cfg.get("SWEEP_VAR")
    vals = cfg.get("SWEEP_VALUES")
    if sv and vals is not None:
        cfg[sv] = list(vals)

    D_ins    = _to_list(cfg["D_IN"])
    P_ins    = _to_list(cfg["P_IN"])
    rpms     = _to_list(cfg["RPM"])
    v_climbs = _to_list(cfg["V_CLIMB"])
    airfoils = _to_list(cfg["AIRFOIL"])

    combos = []
    for D_in, P_in, rpm, v_climb, airfoil in itertools.product(
            D_ins, P_ins, rpms, v_climbs, airfoils):
        combos.append({
            "D_IN": D_in, "P_IN": P_in, "RPM": rpm,
            "V_CLIMB": v_climb, "AIRFOIL": airfoil,
        })
    return combos


def run(config: dict) -> dict:
    """
    Run single-rotor analysis for all input combinations.

    Returns
    -------
    dict:
        "cases"   : list of per-case result dicts
        "figures" : [fig_loading, fig_inflow]
    """
    from aerodynamic_model.single_rotor import solve_single_rotor

    B = config["B"]
    combos = _build_combos(config)

    sweep_var = config.get("SWEEP_VAR")

    # Geometry cache: avoid reloading for the same (D, P, airfoil) pair
    _geo_cache: dict[tuple, tuple] = {}

    cases = []
    for combo in combos:
        D_in, P_in, rpm, v_climb, airfoil = (
            combo["D_IN"], combo["P_IN"], combo["RPM"],
            combo["V_CLIMB"], combo["AIRFOIL"],
        )
        D_m = D_in * _IN
        P_m = P_in * _IN
        key = (D_in, P_in, airfoil, config.get("USE_DAT_DIR"))

        if key not in _geo_cache:
            try:
                geo, polars, src = _load_geo_polars(D_m, P_m, B, airfoil, config)
                _geo_cache[key] = (geo, polars, src)
            except Exception as e:
                warnings.warn(f"Geometry load failed D={D_in} P={P_in}: {e}")
                continue
        geo, polars, src = _geo_cache[key]

        try:
            res = solve_single_rotor(
                geo,
                polars,
                rpm,
                V_climb=v_climb,
                return_radial=True,
            )
        except Exception as e:
            warnings.warn(f"BEMT failed D={D_in} P={P_in} RPM={rpm} Vc={v_climb}: {e}")
            continue

        cases.append({
            "D_IN":     D_in,
            "P_IN":     P_in,
            "RPM":      rpm,
            "V_CLIMB":  v_climb,
            "AIRFOIL":  airfoil,
            "source":   src,
            "T_N":      res["T_N"],
            "Q_Nm":     res["Q_Nm"],
            "P_aero_W": res["P_aero_W"],
            "CT":       res["CT"],
            "CQ":       res["CQ"],
            "J":        res["J"],
            "eta":      res["eta"],
            "r_norm":       res["r"] / geo["R"],
            "dT_dr":        res["dT_dr"],
            "dQ_dr":        res["dQ_dr"],
            "v_i":          res["v_i"],
            "alpha_arr_deg": res["alpha_arr_deg"],
            "F_arr":        res["F_arr"],
            "phi_arr_deg":  res["phi_arr_deg"],
        })

    if not cases:
        print("No cases completed.")
        return {"cases": [], "figures": []}

    make_plots    = config.get("MAKE_PLOTS",    True)
    print_summary = config.get("PRINT_SUMMARY", True)

    # -- Console summary -------------------------------------------------------
    if print_summary:
        print()
        print("=" * 82)
        print(f"  {'D[in]':>6} {'P[in]':>6} {'RPM':>7} {'Vc[m/s]':>8} {'T[N]':>7} "
              f"{'CT':>8} {'CQ':>8} {'J':>6} {'eta':>6} {'source':>10}")
        print("-" * 82)
        for c in cases:
            print(f"  {c['D_IN']:>6.1f} {c['P_IN']:>6.1f} {c['RPM']:>7.0f} {c['V_CLIMB']:>8.2f} "
                  f"{c['T_N']:>7.3f} {c['CT']:>8.5f} {c['CQ']:>8.5f} "
                  f"{c['J']:>6.3f} {c['eta']:>6.3f} {c['source']:>10}")
        print("=" * 82)

    # -- Plots -----------------------------------------------------------------
    figures = []
    if not make_plots:
        return {"cases": cases, "figures": figures}

    colors = cm.tab10(np.linspace(0, 1, max(len(cases), 1)))

    def _label(c):
        parts = []
        if len(set(x["D_IN"]   for x in cases)) > 1: parts.append(f"D={c['D_IN']:.0f}\"")
        if len(set(x["P_IN"]   for x in cases)) > 1: parts.append(f"P={c['P_IN']:.1f}\"")
        if len(set(x["RPM"]    for x in cases)) > 1: parts.append(f"RPM={c['RPM']:.0f}")
        if len(set(x["V_CLIMB"] for x in cases)) > 1: parts.append(f"Vc={c['V_CLIMB']:.1f}")
        if len(set(x["AIRFOIL"] for x in cases)) > 1: parts.append(c["AIRFOIL"])
        if not parts:
            parts.append(f"D={c['D_IN']:.0f}\" P={c['P_IN']:.1f}\" RPM={c['RPM']:.0f}")
        return "  ".join(parts)

    # Figure 1: blade loading dT/dr and dQ/dr
    fig1, axes1 = plt.subplots(1, 2, figsize=(12, 4), sharex=True)
    sweep_title = f" (sweep {sweep_var})" if sweep_var else ""
    fig1.suptitle(f"Blade loading — single rotor{sweep_title}", fontsize=11)
    for c, col in zip(cases, colors):
        lbl = _label(c)
        axes1[0].plot(c["r_norm"], c["dT_dr"], color=col, lw=1.6, label=lbl)
        axes1[1].plot(c["r_norm"], c["dQ_dr"], color=col, lw=1.6, label=lbl)
    axes1[0].set_xlabel("r/R"); axes1[0].set_ylabel("dT/dr  [N/m]")
    axes1[0].set_title("Thrust loading")
    axes1[0].grid(True, alpha=0.4); axes1[0].legend(fontsize=7)
    axes1[1].set_xlabel("r/R"); axes1[1].set_ylabel("dQ/dr  [Nm/m]")
    axes1[1].set_title("Torque loading")
    axes1[1].grid(True, alpha=0.4); axes1[1].legend(fontsize=7)
    plt.tight_layout()
    figures.append(fig1)

    # Figure 2: blade section aerodynamics
    fig2, axes2 = plt.subplots(1, 3, figsize=(14, 4), sharex=True)
    fig2.suptitle(f"Blade section aerodynamics{sweep_title}", fontsize=11)
    for c, col in zip(cases, colors):
        lbl = _label(c)
        axes2[0].plot(c["r_norm"], c["alpha_arr_deg"], color=col, lw=1.6, label=lbl)
        axes2[1].plot(c["r_norm"], c["F_arr"],         color=col, lw=1.6, label=lbl)
        axes2[2].plot(c["r_norm"], c["phi_arr_deg"],   color=col, lw=1.6, label=lbl)
    axes2[0].axhline(14, color="red", lw=0.8, ls="--", label="stall ~14°")
    axes2[0].set_ylabel("AoA [deg]"); axes2[0].set_title("Angle of Attack")
    axes2[1].set_ylabel("F");         axes2[1].set_title("Tip-loss factor (Prandtl)")
    axes2[2].set_ylabel("φ [deg]");   axes2[2].set_title("Inflow angle φ")
    for ax in axes2:
        ax.set_xlabel("r/R"); ax.grid(True, alpha=0.4); ax.legend(fontsize=7)
    plt.tight_layout()
    figures.append(fig2)

    # Figure 3: scalar performance summary : compares the Thrust, Aero Power, Figure of Merit and Power Loading across cases.
    
    _RHO = 1.225
    fig3, axes3 = plt.subplots(1, 4, figsize=(14, 4))
    fig3.suptitle(f"Performance summary — single rotor{sweep_title}", fontsize=11)
    bar_labels = [_label(c) for c in cases]
    x = np.arange(len(cases))
    w = 0.6
    T_vals = [c["T_N"]      for c in cases]
    P_vals = [c["P_aero_W"] for c in cases]
    FM_vals, PL_vals = [], []
    for c in cases:
        R_m = c["D_IN"] * _IN / 2
        A   = _math.pi * R_m ** 2
        fm  = (c["T_N"] ** 1.5 / _math.sqrt(max(2 * _RHO * A, 1e-12))) / max(c["P_aero_W"], 1e-6)
        pl  = c["T_N"] / max(c["P_aero_W"], 1e-6) * 1000 / 9.81
        FM_vals.append(fm)
        PL_vals.append(pl)
    for ax, vals, ylabel, title, color in zip(
        axes3,
        [T_vals, P_vals, FM_vals, PL_vals],
        ["T [N]", "P [W]", "FM", "g/W"],
        ["Thrust", "Aero Power", "Figure of Merit", "Power Loading"],
        ["C0", "C1", "C2", "C3"],
    ):
        ax.bar(x, vals, w, color=color, alpha=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(bar_labels, rotation=20, ha="right", fontsize=7)
        ax.set_ylabel(ylabel); ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.4)
    plt.tight_layout()
    figures.append(fig3)

    # Figure 4: propeller polar (CT, CQ, eta vs J) grouped by RPM
    polar_groups: dict[float, list[dict]] = {}
    for c in cases:
        polar_groups.setdefault(float(c["RPM"]), []).append(c)

    polar_groups = {
        rpm_key: sorted(group, key=lambda case: case["J"])
        for rpm_key, group in polar_groups.items()
        if len(group) >= 2
    }

    if polar_groups:
        fig4, axes4 = plt.subplots(1, 3, figsize=(15, 4), sharex=False)
        fig4.suptitle(f"Propeller polar â€” single rotor{sweep_title}", fontsize=11)
        polar_colors = cm.tab10(np.linspace(0, 1, max(len(polar_groups), 1)))

        for (rpm_key, group), col in zip(sorted(polar_groups.items()), polar_colors):
            base = group[0]
            lbl_parts = [f"RPM={rpm_key:.0f}"]
            if len(set(x["AIRFOIL"] for x in cases)) > 1:
                lbl_parts.append(base["AIRFOIL"])
            if len(set(x["D_IN"] for x in cases)) > 1:
                lbl_parts.append(f'D={base["D_IN"]:.0f}"')
            if len(set(x["P_IN"] for x in cases)) > 1:
                lbl_parts.append(f'P={base["P_IN"]:.1f}"')
            lbl = "  ".join(lbl_parts)

            J_arr = [g["J"] for g in group]
            CT_arr = [g["CT"] for g in group]
            CQ_arr = [g["CQ"] for g in group]
            eta_arr = [g["eta"] for g in group]

            axes4[0].plot(J_arr, CT_arr, color=col, lw=1.8, marker="o", ms=3.5, label=lbl)
            axes4[1].plot(J_arr, CQ_arr, color=col, lw=1.8, marker="o", ms=3.5, label=lbl)
            axes4[2].plot(J_arr, eta_arr, color=col, lw=1.8, marker="o", ms=3.5, label=lbl)

        axes4[0].set_xlabel("J"); axes4[0].set_ylabel("CT");  axes4[0].set_title("Thrust coefficient")
        axes4[1].set_xlabel("J"); axes4[1].set_ylabel("CQ");  axes4[1].set_title("Torque coefficient")
        axes4[2].set_xlabel("J"); axes4[2].set_ylabel("eta"); axes4[2].set_title("Efficiency")
        axes4[2].set_ylim(0, 1)
        for ax in axes4:
            ax.grid(True, alpha=0.4)
            ax.legend(fontsize=7)
        plt.tight_layout()
        figures.append(fig4)

    # Figure 5: scalar outputs vs sweep variable (if sweep mode)
    if sweep_var and config.get("SWEEP_VALUES") is not None:
        x_vals = [c[sweep_var] for c in cases]
        fig5, axes5 = plt.subplots(1, 3, figsize=(14, 4))
        fig5.suptitle(f"Single-rotor performance vs {sweep_var}", fontsize=11)
        axes5[0].plot(x_vals, [c["T_N"]      for c in cases], "o-", lw=1.8)
        axes5[0].set_xlabel(sweep_var); axes5[0].set_ylabel("T  [N]")
        axes5[0].set_title("Thrust"); axes5[0].grid(True, alpha=0.4)

        axes5[1].plot(x_vals, [c["CT"]       for c in cases], "o-", lw=1.8, color="C1")
        axes5[1].set_xlabel(sweep_var); axes5[1].set_ylabel("CT")
        axes5[1].set_title("Thrust coefficient"); axes5[1].grid(True, alpha=0.4)

        axes5[2].plot(x_vals, [c["eta"]      for c in cases], "o-", lw=1.8, color="C2")
        axes5[2].set_xlabel(sweep_var); axes5[2].set_ylabel("eta")
        axes5[2].set_title("Propulsive efficiency"); axes5[2].grid(True, alpha=0.4)

        plt.tight_layout()
        figures.append(fig5)

    return {"cases": cases, "figures": figures}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Single-rotor BEMT analysis")
    parser.add_argument("--diameter-in", type=float, help="propeller diameter [in]")
    parser.add_argument("--pitch-in", type=float, help="propeller pitch [in]")
    parser.add_argument("--blades", type=int, help="blade count")
    parser.add_argument("--rpm", type=float, help="rotational speed [RPM]")
    parser.add_argument("--v-climb", type=float, help="axial climb velocity [m/s]")
    parser.add_argument("--airfoil", help="airfoil name, e.g. clark_y, naca2412")
    parser.add_argument("--sweep-var", choices=["D_IN", "P_IN", "RPM", "V_CLIMB"], help="sweep one variable")
    parser.add_argument("--sweep-values", help="comma-separated values for --sweep-var, e.g. 4000,5000,6000")
    parser.add_argument("--no-plots", action="store_true", help="skip matplotlib figures, print summary only")
    args = parser.parse_args()

    config = dict(CONFIG)
    if args.diameter_in is not None: config["D_IN"] = args.diameter_in
    if args.pitch_in is not None: config["P_IN"] = args.pitch_in
    if args.blades is not None: config["B"] = args.blades
    if args.rpm is not None: config["RPM"] = args.rpm
    if args.v_climb is not None: config["V_CLIMB"] = args.v_climb
    if args.airfoil is not None: config["AIRFOIL"] = args.airfoil
    if args.sweep_var is not None: config["SWEEP_VAR"] = args.sweep_var
    if args.sweep_values is not None: config["SWEEP_VALUES"] = [float(v) for v in args.sweep_values.split(",")]
    if args.no_plots: config["MAKE_PLOTS"] = False

    out = run(config)
    if out.get("figures"):
        plt.show()
