"""
airfoil.py — Airfoil polar engine (Phase 1)
============================================
Returns Reynolds-number-corrected Cl(α) and Cd(α) polars using NeuralFoil.

NeuralFoil is a physics-informed ML model trained on XFoil data that:
  - Covers Re 10k–5M, α from -180° to 180°
  - Requires no external binaries (pure Python/NumPy)
  - Guarantees convergence (unlike XFoil)

Supported airfoils
------------------
  'clark_y'   : NACA/Clark Y — default, covers >80% of small UAV propellers
  'naca4412'  : Alternative for higher-pitch designs
  Custom      : Pass NACA 4-digit string, e.g. 'naca2412'

References
----------
NeuralFoil: github.com/peterdsharpe/NeuralFoil
  Peter Sharpe, "NeuralFoil: An airfoil aerodynamics analysis tool using
  physics-informed machine learning," 2023.
"""

from __future__ import annotations

import hashlib
import pickle
import warnings
from functools import lru_cache
from pathlib import Path

import numpy as np

# Disk cache directory: repo_root/data/polars_cache/
_POLARS_CACHE_DIR = Path(__file__).parent.parent / "data" / "polars_cache"

# Airfoil name → NeuralFoil-compatible identifier
_AIRFOIL_MAP = {
    "clark_y": "clarky",
    "clarky": "clarky",
    "naca4412": "naca4412",
    "naca2412": "naca2412",
    "naca0012": "naca0012",
}

# Default operating range
_DEFAULT_RE = (
    [10_000, 20_000, 50_000]                   # coarse below 100k
    + list(range(100_000, 1_010_000, 10_000))  # 100k → 1M in 10k steps (91 points)
)
_DEFAULT_ALPHA = np.linspace(-5, 25, 61)  # degrees


def _polars_cache_key(airfoil_key: str, Re_values: list, alpha_range: tuple, n_alpha: int) -> str:
    """Stable hash key for a polars computation."""
    payload = f"{airfoil_key}|{sorted(Re_values)}|{alpha_range}|{n_alpha}"
    return hashlib.md5(payload.encode()).hexdigest()


_POLARS_MEMORY_CACHE: dict = {}


def get_airfoil_polars(
    airfoil: str = "clark_y",
    Re_values: list[float] | None = None,
    alpha_range: tuple[float, float] = (-5, 25),
    n_alpha: int = 61,
    use_disk_cache: bool = True,
) -> dict:
    """Return Cl and Cd polars over a range of Reynolds numbers and AoA.

    Parameters
    ----------
    airfoil : str
        Airfoil identifier. Default 'clark_y'. Accepts aliases defined in
        _AIRFOIL_MAP or any NACA 4/5-digit string (e.g. 'naca4412').
    Re_values : list of float, optional
        Reynolds numbers to evaluate. Defaults to [20k, 50k, 100k, 200k, 500k].
    alpha_range : (float, float)
        (alpha_min, alpha_max) in degrees. Default (-5, 25).
    n_alpha : int
        Number of alpha points. Default 61.

    Returns
    -------
    dict with keys:
        airfoil   : str — resolved airfoil name used
        Re        : ndarray (n_Re,) — Reynolds numbers
        alpha_deg : ndarray (n_alpha,) — angle of attack [deg]
        Cl        : ndarray (n_Re, n_alpha) — lift coefficient
        Cd        : ndarray (n_Re, n_alpha) — drag coefficient
        Cl_alpha  : ndarray (n_Re,) — lift curve slope [1/deg] at α=0
    """
    if Re_values is None:
        Re_values = _DEFAULT_RE

    alpha_arr = np.linspace(alpha_range[0], alpha_range[1], n_alpha)
    Re_arr = np.asarray(Re_values, dtype=float)

    airfoil_key = _resolve_airfoil_name(airfoil)

    # ── Memory cache check (fastest — survives across requests in same process) ─
    mem_key = _polars_cache_key(airfoil_key, list(Re_values), alpha_range, n_alpha)
    if mem_key in _POLARS_MEMORY_CACHE:
        return _POLARS_MEMORY_CACHE[mem_key]
    # ─────────────────────────────────────────────────────────────────────────

    # ── Disk cache check ──────────────────────────────────────────────────────
    if use_disk_cache:
        cache_file = _POLARS_CACHE_DIR / f"{airfoil_key}_{mem_key}.pkl"
        if cache_file.exists():
            with open(cache_file, "rb") as f:
                result = pickle.load(f)
            _POLARS_MEMORY_CACHE[mem_key] = result
            return result
    # ─────────────────────────────────────────────────────────────────────────

    Cl_table = np.zeros((len(Re_arr), n_alpha))
    Cd_table = np.zeros((len(Re_arr), n_alpha))

    try:
        import neuralfoil as nf  # type: ignore
        _use_neuralfoil = True
    except ImportError:
        warnings.warn(
            "neuralfoil not installed. Falling back to thin-airfoil theory + Oswald drag. "
            "Install with: pip install neuralfoil",
            UserWarning,
            stacklevel=2,
        )
        _use_neuralfoil = False

    for i, Re in enumerate(Re_arr):
        if _use_neuralfoil:
            Cl_table[i], Cd_table[i] = _query_neuralfoil(airfoil_key, Re, alpha_arr)
        else:
            Cl_table[i], Cd_table[i] = _thin_airfoil_fallback(alpha_arr)

    # Lift curve slope: dCl/dα around α = 0 [1/deg]
    idx_zero = np.argmin(np.abs(alpha_arr))
    # Use ±3 points around zero for finite difference
    lo = max(0, idx_zero - 3)
    hi = min(n_alpha - 1, idx_zero + 3)
    Cl_alpha = np.gradient(Cl_table[:, lo:hi+1], axis=1).mean(axis=1) / (
        alpha_arr[1] - alpha_arr[0]
    )

    result = {
        "airfoil": airfoil_key,
        "Re": Re_arr,
        "alpha_deg": alpha_arr,
        "Cl": Cl_table,
        "Cd": Cd_table,
        "Cl_alpha": Cl_alpha,
    }

    # ── Save to memory + disk cache ───────────────────────────────────────────
    _POLARS_MEMORY_CACHE[mem_key] = result
    if use_disk_cache:
        try:
            _POLARS_CACHE_DIR.mkdir(parents=True, exist_ok=True)
            disk_file = _POLARS_CACHE_DIR / f"{airfoil_key}_{mem_key}.pkl"
            with open(disk_file, "wb") as f:
                pickle.dump(result, f, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            warnings.warn(f"Polars disk cache write failed: {e}")
    # ─────────────────────────────────────────────────────────────────────────

    return result


def interpolate_polar(
    polars: dict,
    Re: float,
    alpha_deg: float,
) -> tuple[float, float]:
    """Interpolate Cl and Cd at a given Re and alpha.

    Parameters
    ----------
    polars : dict
        Output of :func:`get_airfoil_polars`.
    Re : float
        Reynolds number.
    alpha_deg : float
        Angle of attack [deg].

    Returns
    -------
    Cl : float
    Cd : float
    """
    Re_table = polars["Re"]
    alpha_table = polars["alpha_deg"]
    Cl_table = polars["Cl"]
    Cd_table = polars["Cd"]

    # Clamp to table bounds
    Re_c = float(np.clip(Re, Re_table[0], Re_table[-1]))
    alpha_c = float(np.clip(alpha_deg, alpha_table[0], alpha_table[-1]))

    # Bilinear interpolation
    i_re = np.searchsorted(Re_table, Re_c, side="right") - 1
    i_re = int(np.clip(i_re, 0, len(Re_table) - 2))
    t_re = (Re_c - Re_table[i_re]) / (Re_table[i_re + 1] - Re_table[i_re])

    i_a = np.searchsorted(alpha_table, alpha_c, side="right") - 1
    i_a = int(np.clip(i_a, 0, len(alpha_table) - 2))
    t_a = (alpha_c - alpha_table[i_a]) / (alpha_table[i_a + 1] - alpha_table[i_a])

    def _blerp(arr):
        v00 = arr[i_re, i_a]
        v01 = arr[i_re, i_a + 1]
        v10 = arr[i_re + 1, i_a]
        v11 = arr[i_re + 1, i_a + 1]
        return (1 - t_re) * ((1 - t_a) * v00 + t_a * v01) + t_re * ((1 - t_a) * v10 + t_a * v11)

    return float(_blerp(Cl_table)), float(_blerp(Cd_table))


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _resolve_airfoil_name(name: str) -> str:
    """Resolve user-supplied airfoil name to NeuralFoil-compatible string."""
    lower = name.lower().replace("-", "").replace("_", "").replace(" ", "")
    return _AIRFOIL_MAP.get(lower, lower)


def _query_neuralfoil(
    airfoil: str,
    Re: float,
    alpha_arr: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Query NeuralFoil for Cl, Cd arrays at given Re over alpha sweep."""
    import neuralfoil as nf  # type: ignore
    import aerosandbox as asb  # type: ignore

    af = asb.Airfoil(airfoil)
    results = nf.get_aero_from_airfoil(
        airfoil=af,
        alpha=alpha_arr,
        Re=Re,
        model_size="large",
    )
    Cl = np.asarray(results["CL"])
    Cd = np.asarray(results["CD"])
    return Cl, Cd


def _thin_airfoil_fallback(alpha_arr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Thin-airfoil theory + stall model — used when NeuralFoil is unavailable.

    Clark Y characteristics:
      - Zero-lift angle α_L0 ≈ -4°
      - Stall at α ≈ 14° (Cl_max ≈ 1.25 for Re ~100k)
      - Post-stall: flat-plate model Cl = 2·sin(α)·cos(α), Cd = 2·sin²(α)
    """
    alpha_rad = np.radians(alpha_arr)
    alpha_L0 = np.radians(-4.0)
    alpha_stall = np.radians(14.0)
    Cl_max = 1.25

    # Linear pre-stall
    Cl_linear = 2 * np.pi * (alpha_rad - alpha_L0)

    # Post-stall: flat-plate approximation
    Cl_flat = 2 * np.sin(alpha_rad) * np.cos(alpha_rad)

    # Blend: use linear below stall, flat-plate above
    stalled = alpha_rad > alpha_stall
    Cl = np.where(stalled, Cl_flat, Cl_linear)

    # Drag: pre-stall parasitic + induced; post-stall flat-plate
    Cd_prestall = 0.012 + 0.005 * Cl_linear ** 2
    Cd_flat = 2 * np.sin(alpha_rad) ** 2
    Cd = np.where(stalled, Cd_flat, Cd_prestall)
    Cd = np.clip(Cd, 0.005, 2.0)
    return Cl, Cd
