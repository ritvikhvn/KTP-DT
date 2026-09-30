"""Dryden Wind Turbulence Model (Discrete) — MIL-F-8785C, Metric.

Discrete Dryden wind turbulence model.
mask parameters exactly:
  - Units:                   Metric (MKS)
  - Specification:           MIL-F-8785C
  - Model type:              Discrete Dryden (+q +r)  [linear gusts only for multirotor]
  - W20:                     wind speed at 6 m altitude [m/s]
  - wind_heading_deg:        compass heading wind comes FROM, degrees clockwise from North
  - horizontal_wind_speed_ms: mean horizontal wind speed [m/s]
  - vertical_wind_speed_ms:  mean vertical wind speed [m/s], +ve = down (NED)
  - dt_filt:                 filter update period [s] — how often the Dryden filter steps.
                             Independent of Ts (physics timestep = 1/sampling_rate_hz).
                             dt_filt >= Ts. Filter holds last output between updates.
  - seeds:                   [ug, vg, wg, pg] noise seeds

Timing relationship:
  Ts      = 1 / sampling_rate_hz   (physics tick period, from sim config)
  dt_filt = filter update period   (from dryden config, e.g. 0.1 s = 10 Hz updates)
  dt_filt >= Ts always.
  update() is called every physics tick with Ts. The filter only steps internally
  when accumulated time >= dt_filt, otherwise holds last output.

Discrete first-order shaping filter (MIL-F-8785C, low altitude):

    x[n] = (1 - V/L * dt_filt) * x[n-1] + sqrt(2*V/L * dt_filt) * sigma * eta[n]

Scale lengths (h in feet, then converted to metres):
    Lw = h_ft / FT_PER_Myes
    Lu = Lv = (h_ft / (0.177 + 0.000823*h_ft)^1.2) / FT_PER_M

Turbulence intensities (h in feet, W20 in m/s):
    sigma_w = 0.1 * W20
    sigma_u = sigma_v = sigma_w / (0.177 + 0.000823*h_ft)^0.4

Output:
    v_gust_ned (3,) in NED frame [m/s]
"""
from __future__ import annotations

import numpy as np


_FT_PER_M = 3.28084
_MIN_ALTITUDE_M = 0.5   # floor to avoid Lw -> 0
_MIN_AIRSPEED_MS = 0.5  # floor to avoid div-zero in filter alpha


class DrydenTurbulence:
    """Stateful discrete Dryden turbulence model (MIL-F-8785C, Metric).

    Args:
        W20:                      wind speed at 6 m (20 ft) altitude [m/s]. Controls gust intensity.
        wind_heading_deg:         compass heading wind comes FROM, degrees clockwise from North [deg].
        horizontal_wind_speed_ms: mean horizontal wind speed [m/s].
        vertical_wind_speed_ms:   mean vertical wind speed [m/s], +ve = down (NED).
        dt_filt:                  filter update period [s]. Independent of physics Ts.
                                  Filter steps only when accumulated physics time >= dt_filt.
        seeds:                    4-element list [ug, vg, wg, pg] for independent RNG streams.
                                  Defaults to [23341, 23342, 23343, 23344].
    """

    def __init__(
        self,
        W20: float = 15.0,
        wind_heading_deg: float = 0.0,
        horizontal_wind_speed_ms: float = 0.0,
        vertical_wind_speed_ms: float = 0.0,
        dt_filt: float = 0.1,
        seeds: list[int] | None = None,
        seed_seq: np.random.SeedSequence | None = None,
    ):
        self.W20 = W20
        self.wind_heading_deg = wind_heading_deg
        self.dt_filt = dt_filt
        self._horizontal_wind_speed_ms = horizontal_wind_speed_ms
        self._vertical_wind_speed_ms = vertical_wind_speed_ms

        heading_rad = np.deg2rad(wind_heading_deg)
        self._wind_ned = np.array([
            horizontal_wind_speed_ms * np.cos(heading_rad),
            horizontal_wind_speed_ms * np.sin(heading_rad),
            vertical_wind_speed_ms,
        ])

        # Snapshot startup wind values so runtime overrides can revert to config.
        self._startup_wind = {
            "speed_ms": horizontal_wind_speed_ms,
            "heading_deg": wind_heading_deg,
            "vertical_ms": vertical_wind_speed_ms,
            "W20": W20,
        }

        # When seed_seq provided (from master sim seed), derive 3 independent child RNGs.
        # Otherwise fall back to explicit per-channel integer seeds list.
        # Snapshot the 3 seed inputs so reset() can re-create identical RNGs (see _seed_rngs).
        if seed_seq is not None:
            self._rng_seeds = seed_seq.spawn(3)
        else:
            if seeds is None:
                seeds = [23341, 23342, 23343, 23344]
            # Independent RNG per channel.
            # seeds[3] = pg (angular gust) — not used for linear-only model.
            self._rng_seeds = seeds[:3]
        self._seed_rngs()

        # Filter states
        self._ug = 0.0
        self._vg = 0.0
        self._wg = 0.0

        # Last output held between filter updates (starts at mean wind, no stochastic component yet)
        self._v_gust_ned = self._wind_ned.copy()

        # Accumulated physics time since last filter step
        self._t_accum = 0.0

    def _seed_rngs(self) -> None:
        """(Re)create the 3 per-channel RNGs from the snapshot seeds. Deterministic:
        default_rng() does not mutate a SeedSequence, so this reproduces the identical
        generators every call — used by both __init__ and reset()."""
        self._rng_u = np.random.default_rng(self._rng_seeds[0])
        self._rng_v = np.random.default_rng(self._rng_seeds[1])
        self._rng_w = np.random.default_rng(self._rng_seeds[2])

    def reset(self) -> None:
        self._ug = 0.0
        self._vg = 0.0
        self._wg = 0.0
        self._v_gust_ned = self._wind_ned.copy()
        self._t_accum = 0.0
        self._seed_rngs()  # restart RNG streams from seed — matches sensor reset() behavior

    def set_wind(
        self,
        *,
        speed_ms: float | None = None,
        heading_deg: float | None = None,
        vertical_ms: float | None = None,
        W20: float | None = None,
    ) -> None:
        """Mutate wind params live. Partial update — only provided keys change.
        Recomputes mean NED wind vector. Filter state preserved."""
        if speed_ms is not None:
            self._horizontal_wind_speed_ms = float(speed_ms)
        if heading_deg is not None:
            self.wind_heading_deg = float(heading_deg)
        if vertical_ms is not None:
            self._vertical_wind_speed_ms = float(vertical_ms)
        if W20 is not None:
            self.W20 = float(W20)
        heading_rad = np.deg2rad(self.wind_heading_deg)
        self._wind_ned = np.array([
            self._horizontal_wind_speed_ms * np.cos(heading_rad),
            self._horizontal_wind_speed_ms * np.sin(heading_rad),
            self._vertical_wind_speed_ms,
        ])

    def reset_wind_to_startup(self) -> None:
        """Restore wind params to the values read from startup_config.json."""
        s = self._startup_wind
        self.set_wind(
            speed_ms=s["speed_ms"],
            heading_deg=s["heading_deg"],
            vertical_ms=s["vertical_ms"],
            W20=s["W20"],
        )

    def update(
        self,
        x_ned: np.ndarray,
        dcm_be: np.ndarray,
        airspeed: float,
        Ts: float,
    ) -> np.ndarray:
        """Called every physics tick (period = Ts = 1/sampling_rate_hz).

        Filter steps only when accumulated time >= dt_filt, otherwise
        returns the last computed gust vector unchanged.

        Args:
            x_ned:    position NED [m]; altitude = -x_ned[2]
            dcm_be:   earth->body DCM (body->earth = dcm_be.T)
            airspeed: |v_body| [m/s]
            Ts:       physics timestep [s] = 1/sampling_rate_hz

        Returns:
            v_gust_ned (3,) [m/s] in NED frame
        """
        self._t_accum += Ts

        if self._t_accum < self.dt_filt:
            return self._v_gust_ned   # hold last output

        # Filter step — use actual accumulated time as the filter dt
        dt = self._t_accum
        self._t_accum = 0.0

        altitude_m = max(-float(x_ned[2]), _MIN_ALTITUDE_M)
        h_ft = altitude_m * _FT_PER_M
        V = max(airspeed, _MIN_AIRSPEED_MS)

        # Scale lengths (MIL-F-8785C, h in feet → metres)
        phi = 0.177 + 0.000823 * h_ft
        Lw = h_ft / _FT_PER_M
        Lu = Lv = (h_ft / (phi ** 1.2)) / _FT_PER_M

        # Turbulence intensities
        sigma_w = 0.1 * self.W20
        sigma_uv = sigma_w / (phi ** 0.4)

        # Discrete filter: x[n] = (1 - V/L*dt)*x[n-1] + sqrt(2*V/L*dt)*sigma*eta
        def _step(x_prev: float, L: float, sigma: float, rng: np.random.Generator) -> float:
            alpha = V / L * dt
            return (1.0 - alpha) * x_prev + np.sqrt(2.0 * alpha) * sigma * rng.standard_normal()

        self._ug = _step(self._ug, Lu, sigma_uv, self._rng_u)
        self._vg = _step(self._vg, Lv, sigma_uv, self._rng_v)
        self._wg = _step(self._wg, Lw, sigma_w,  self._rng_w)

        # Gust in body frame → NED
        v_gust_body = np.array([self._ug, self._vg, self._wg])
        v_gust_ned = dcm_be.T @ v_gust_body

        if self.wind_heading_deg != 0.0:
            v_gust_ned = _rotate_ned_yaw(v_gust_ned, self.wind_heading_deg)

        self._v_gust_ned = self._wind_ned + v_gust_ned
        return self._v_gust_ned


def _rotate_ned_yaw(v_ned: np.ndarray, yaw_deg: float) -> np.ndarray:
    """Rotate NED vector by yaw_deg degrees clockwise from North."""
    psi = np.deg2rad(yaw_deg)
    c, s = np.cos(psi), np.sin(psi)
    R = np.array([
        [ c, s, 0.0],
        [-s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return R @ v_ned
