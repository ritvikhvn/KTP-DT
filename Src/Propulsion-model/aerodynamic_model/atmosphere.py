"""
atmosphere.py — ISA atmosphere model
=====================================
Provides sea-level air properties and an ISA altitude-correction class.

Module-level constants (sea level, ISA standard):
  RHO_SL  : float  — air density [kg/m³]  = 1.225
  NU_SL   : float  — kinematic viscosity [m²/s] = 1.5e-5

For all current solvers, import RHO_SL / NU_SL directly.
Use ISAAtmosphere for altitude-aware computations (Phase 5+).

References
----------
ICAO (1993) Manual of the ICAO Standard Atmosphere (3rd ed.)
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Sea-level constants — import these directly in solver modules
# ---------------------------------------------------------------------------
RHO_SL: float = 1.225    # kg/m³  ISA sea-level density
NU_SL:  float = 1.5e-5   # m²/s   ISA sea-level kinematic viscosity (rounded standard)


# ---------------------------------------------------------------------------
# ISA altitude model
# ---------------------------------------------------------------------------
class ISAAtmosphere:
    """International Standard Atmosphere (troposphere only, 0–11 km).

    Parameters
    ----------
    altitude_m : float
        Geometric altitude above sea level [m]. Must be in [0, 11000].

    Attributes
    ----------
    rho : float   Air density [kg/m³]
    nu  : float   Kinematic viscosity [m²/s]
    T   : float   Temperature [K]
    p   : float   Pressure [Pa]
    """

    _T0  = 288.15    # K      ISA sea-level temperature
    _L   = 0.0065    # K/m    ISA lapse rate
    _P0  = 101325.0  # Pa     ISA sea-level pressure
    _g   = 9.80665   # m/s²   standard gravity
    _R   = 287.058   # J/(kg·K) specific gas constant for dry air
    _mu0 = 1.789e-5  # Pa·s   sea-level dynamic viscosity (Sutherland reference)

    def __init__(self, altitude_m: float = 0.0) -> None:
        if not 0.0 <= altitude_m <= 11000.0:
            raise ValueError(
                f"altitude_m={altitude_m} is outside troposphere range [0, 11000 m]"
            )
        self._altitude = altitude_m
        T, p, rho = self._isa(altitude_m)
        self.T   = T
        self.p   = p
        self.rho = rho
        # Sutherland's law for dynamic viscosity
        S  = 110.4  # K  Sutherland constant
        mu = self._mu0 * (T / self._T0) ** 1.5 * (self._T0 + S) / (T + S)
        self.nu = mu / rho

    def _isa(self, h: float) -> tuple[float, float, float]:
        T   = self._T0 - self._L * h
        exp = self._g / (self._R * self._L)
        p   = self._P0 * (T / self._T0) ** exp
        rho = p / (self._R * T)
        return T, p, rho

    def __repr__(self) -> str:
        return (
            f"ISAAtmosphere(altitude={self._altitude:.0f} m, "
            f"rho={self.rho:.4f} kg/m³, nu={self.nu:.3e} m²/s)"
        )
