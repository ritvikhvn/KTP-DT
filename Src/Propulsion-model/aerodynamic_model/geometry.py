"""APC blade geometry lookup."""
from __future__ import annotations

import re
from pathlib import Path
import numpy as np
from .atmosphere import RHO_SL

_DEFAULT_APC_DIR = Path(__file__).parents[1] / "data" / "apc_geo" / "PE0-FILES_WEB"


def get_blade_geometry(D_m, P_m, B, design_rpm, design_thrust_N, n_stations=20, apc_data_dir=None, rho=RHO_SL):
    del design_rpm, design_thrust_N, rho
    apc_dir = Path(apc_data_dir) if apc_data_dir is not None else _DEFAULT_APC_DIR
    result = _try_apc_lookup(float(D_m), float(P_m), int(n_stations), apc_dir)
    if result is None:
        raise NotImplementedError(f"No APC geometry matches D={D_m:.3f} m, P={P_m:.3f} m; supply blade data")
    r_norm, chord, twist, filename = result
    return {"r": r_norm * D_m / 2.0, "r_norm": r_norm, "chord": chord, "twist_deg": twist, "R": D_m / 2.0, "B": B, "source": "apc", "apc_file": filename}


def _try_apc_lookup(D_m, P_m, n_stations, apc_dir):
    if not apc_dir.exists(): return None
    D_in, P_in = D_m / 0.0254, P_m / 0.0254
    best, error = None, float("inf")
    for path in apc_dir.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in {".peo", ".pe0"}: continue
        d, p = _parse_apc_filename(path.name)
        if d is None: continue
        err = abs(d - D_in) / max(D_in, 1e-9) + abs(p - P_in) / max(P_in, 1e-9)
        if err < error: best, error = path, err
    if best is None or error > 0.06: return None
    r_norm, chord, _ = _parse_peo_file(best)
    if r_norm is None: return None
    rout = np.linspace(r_norm[0], r_norm[-1], n_stations)
    twist = np.degrees(np.arctan(P_m / (2.0 * np.pi * rout * D_m / 2.0 + 1e-9)))
    return rout, np.interp(rout, r_norm, chord), twist, best.name


def _parse_apc_filename(filename):
    match = re.search(r"(\d+(?:\.\d+)?)[xX](\d+(?:\.\d+)?)", filename)
    if not match: return None, None
    diameter, raw = float(match.group(1)), match.group(2)
    if "." not in raw and len(raw) >= 2 and float(raw) > diameter:
        pitch = float(raw[0]) + float(raw[1:]) / 10 ** len(raw[1:])
    else: pitch = float(raw)
    return diameter, pitch


def _parse_peo_file(path):
    try: lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError: return None, None, None
    radius = None; rows = []
    for line in lines:
        parts = line.strip().split()
        if line.strip().upper().startswith("RADIUS:"):
            try: radius = float(parts[1])
            except (IndexError, ValueError): pass
        try:
            station, chord, beta = float(parts[0]), float(parts[1]), float(parts[2])
        except (IndexError, ValueError): continue
        if chord > 0: rows.append((station, chord, beta))
    if len(rows) < 3: return None, None, None
    values = np.asarray(rows); radius = radius or float(values[:, 0].max())
    r = values[:, 0] / radius; mask = (r > 0) & (r <= 1)
    order = np.argsort(r[mask])
    return r[mask][order], values[:, 1][mask][order] * 0.0254, values[:, 2][mask][order]


def print_geometry_summary(geo):
    print(f"Source: {geo['source']} ({geo['apc_file']})")
    print(f"Diameter: {2 * geo['R'] / 0.0254:.1f} in; blades: {geo['B']}; stations: {len(geo['r'])}")
