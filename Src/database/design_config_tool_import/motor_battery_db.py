"""Motor and battery curve loader."""
from __future__ import annotations

import os
import pickle
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from .config import config


CACHE_DIR = Path(__file__).parent / "cache"
MOTOR_CACHE = CACHE_DIR / "motor_database.pkl"
BATT_CACHE = CACHE_DIR / "battery_database.pkl"


@dataclass
class MotorRecord:
    name: str
    type: str
    index: int
    mass: float
    prop_radius: float
    throttle: np.ndarray
    voltage_v: np.ndarray
    current_a: np.ndarray
    power_w: np.ndarray
    rpm: np.ndarray
    torque_nm: np.ndarray
    thrust_g: np.ndarray
    efficiency_gw: np.ndarray

_THROTTLE_GRID = np.linspace(0.0, 1.0, 101)


@dataclass
class MotorLookup:
    """Pre-sampled motor curves on a fixed 101-point throttle grid (0.00…1.00).

    Built once at startup from MotorRecord. All arrays length 101.
    Use interp_* methods for PWM→thrust/torque/power lookup each tick.
    """
    name: str
    throttle: np.ndarray    # shape (101,) — fixed grid 0.00…1.00
    thrust_g: np.ndarray    # shape (101,)
    torque_nm: np.ndarray   # shape (101,)
    power_w: np.ndarray     # shape (101,)
    current_a: np.ndarray   # shape (101,)
    rpm: np.ndarray         # shape (101,) — motor RPM vs throttle (vibration harmonics)

    @staticmethod
    def from_record(record: "MotorRecord") -> "MotorLookup":
        return MotorLookup(
            name=record.name,
            throttle=_THROTTLE_GRID.copy(),
            thrust_g=np.interp(_THROTTLE_GRID, record.throttle, record.thrust_g),
            torque_nm=np.interp(_THROTTLE_GRID, record.throttle, record.torque_nm),
            power_w=np.interp(_THROTTLE_GRID, record.throttle, record.power_w),
            current_a=np.interp(_THROTTLE_GRID, record.throttle, record.current_a),
            rpm=np.interp(_THROTTLE_GRID, record.throttle, record.rpm),
        )

    # PERF (behaviour-identical, no flight-dynamics change): the curves are sampled
    # on the FIXED uniform grid _THROTTLE_GRID = linspace(0,1,101) — 101 points, even
    # spacing 0.01. For a uniform grid the interpolation index is just throttle*100,
    # so we compute it directly instead of calling np.interp (which runs searchsorted
    # + an iscomplexobj check on every call). These run 16x per tick (4 rotors x
    # thrust/torque/power/current) and dominated the tick once RK4 was JIT'd.
    #
    # _interp_uniform performs the SAME piecewise-linear interpolation as np.interp,
    # matched to ~2e-13 (float round-off). Same endpoint clamping too: np.interp pins
    # out-of-range x to fp[0]/fp[-1], and so does this. Callers (compute_motor_forces)
    # already clip throttle to [0,1] via pwm_to_throttle, so range is guaranteed anyway.
    # => identical thrust/torque/power → identical sim output under ArduPilot.
    @staticmethod
    def _interp_uniform(throttle: float, fp: np.ndarray) -> float:
        pos = throttle * 100.0          # index into the 101-point grid (100 intervals)
        if pos <= 0.0:
            return float(fp[0])
        if pos >= 100.0:
            return float(fp[100])
        i = int(pos)                    # lower sample index
        frac = pos - i                  # fractional position within the interval
        return float(fp[i] + frac * (fp[i + 1] - fp[i]))

    def interp_thrust_g(self, throttle: float) -> float:
        return self._interp_uniform(throttle, self.thrust_g)

    def interp_torque_nm(self, throttle: float) -> float:
        return self._interp_uniform(throttle, self.torque_nm)

    def interp_power_w(self, throttle: float) -> float:
        return self._interp_uniform(throttle, self.power_w)

    def interp_current_a(self, throttle: float) -> float:
        return self._interp_uniform(throttle, self.current_a)

    def interp_rpm(self, throttle: float) -> float:
        return self._interp_uniform(throttle, self.rpm)


@dataclass
class BatteryRecord:
    name: str
    index: int
    mass: float
    nominal_voltage: float
    usable_energy_wh: float
    quantity: int = 1

    @property
    def total_mass(self) -> float:
        return self.mass * self.quantity

    @property
    def total_energy_wh(self) -> float:
        return self.usable_energy_wh * self.quantity


_memo: dict = {}


def load_databases(force_reload: bool = False) -> tuple[list[MotorRecord], list[BatteryRecord]]:
    """Return (motors, batteries). Caches in-memory + to pkl file."""
    cfg = config()
    db_file = Path(cfg["db_file"])

    if not force_reload and db_file.exists() and MOTOR_CACHE.exists():
        if db_file.stat().st_mtime > MOTOR_CACHE.stat().st_mtime:
            print("[motor_battery_db] SQLite newer than cache — reloading...")
            force_reload = True

    if not force_reload and _memo:
        return _memo["motors"], _memo["batteries"]

    if not force_reload and MOTOR_CACHE.exists() and BATT_CACHE.exists():
        print("[motor_battery_db] Loading from pkl cache...")
        with open(MOTOR_CACHE, "rb") as f:
            motors = pickle.load(f)
        with open(BATT_CACHE, "rb") as f:
            batteries = pickle.load(f)
        _memo["motors"], _memo["batteries"] = motors, batteries
        return motors, batteries

    if not db_file.exists():
        raise FileNotFoundError(f"SQLite database not found: {db_file}")

    print(f"[motor_battery_db] Loading from SQLite: {db_file}")
    motors = _read_motors(db_file)
    batteries = _read_batteries(db_file)

    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    with open(MOTOR_CACHE, "wb") as f:
        pickle.dump(motors, f)
    with open(BATT_CACHE, "wb") as f:
        pickle.dump(batteries, f)
    print(f"[motor_battery_db] Loaded {len(motors)} motors, {len(batteries)} batteries. Cached to {CACHE_DIR}")

    _memo["motors"], _memo["batteries"] = motors, batteries
    return motors, batteries


def _read_motors(db_file: Path) -> list[MotorRecord]:
    with sqlite3.connect(str(db_file)) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT m.motor_name, m.motor_type, m.mass_kg, m.prop_radius_m, "
            "p.throttle_normalized, p.voltage_v, p.current_a, p.power_w, "
            "p.rpm, p.torque_nm, p.thrust_g, p.efficiency_g_per_w "
            "FROM motors m "
            "JOIN motor_performance p ON m.motor_id = p.motor_id "
            "ORDER BY m.motor_id, p.throttle_normalized"
        )
        rows = cur.fetchall()

    # Group by motor_name preserving insertion order
    grouped: dict[str, list[tuple]] = {}
    for r in rows:
        grouped.setdefault(r[0], []).append(r)

    motors: list[MotorRecord] = []
    idx = 0
    for name, group in grouped.items():
        motor_type = group[0][1]
        mass = float(group[0][2] or 0)
        prop_rad = float(group[0][3] or 0)

        throttle = np.array([_d(r[4]) for r in group])
        voltage_v = np.array([_d(r[5]) for r in group])
        current_a = np.array([_d(r[6]) for r in group])
        power_w = np.array([_d(r[7]) for r in group])
        rpm = np.array([_d(r[8]) for r in group])
        torque_nm = np.array([_d(r[9]) for r in group])
        thrust_g = np.array([_d(r[10]) for r in group])
        eff_gw = np.array([_d(r[11]) for r in group])

        # Ensure throttle monotonically increasing (np.interp requirement)
        order = np.argsort(throttle)
        throttle = throttle[order]
        voltage_v = voltage_v[order]
        current_a = current_a[order]
        power_w = power_w[order]
        rpm = rpm[order]
        torque_nm = torque_nm[order]
        thrust_g = thrust_g[order]
        eff_gw = eff_gw[order]

        motors.append(
            MotorRecord(
                name=name,
                type=motor_type,
                index=idx,
                mass=mass,
                prop_radius=prop_rad,
                throttle=throttle,
                voltage_v=voltage_v,
                current_a=current_a,
                power_w=power_w,
                rpm=rpm,
                torque_nm=torque_nm,
                thrust_g=thrust_g,
                efficiency_gw=eff_gw,
            )
        )
        idx += 1
    return motors


def _read_batteries(db_file: Path) -> list[BatteryRecord]:
    with sqlite3.connect(str(db_file)) as conn:
        cur = conn.cursor()
        cur.execute("SELECT battery_name,total_mass_kg,nominal_voltage_battery,charge_ah,max_voltage_battery,min_voltage_battery,landing_voltage_battery FROM batteries")
        rows = cur.fetchall()

    return [
        BatteryRecord(
            name=r[0],
            index=i,
            mass=_d(r[1]),
            nominal_voltage=_d(r[2]),
            usable_energy_wh=_d(r[3] * r[2] * ((r[4] - r[6]) / (r[4] - r[5]) if r[4] > r[5] else 0.0)),
        )
        for i, r in enumerate(rows)
    ]


def _d(x) -> float:
    return 0.0 if x is None else float(x)


def motor_battery_selection(
    motor_name: str,
    battery_name: Optional[str] = None,
    battery_quantity: int = 1,
    motor_type: Optional[str] = None,
) -> tuple[MotorRecord, Optional[BatteryRecord]]:
    """Name-based lookup."""
    motors, batteries = load_databases()

    matches = [m for m in motors if m.name.lower() == motor_name.lower()]
    if motor_type:
        matches = [m for m in matches if m.type.lower() == motor_type.lower()]
    if not matches:
        names = sorted({m.name for m in motors})
        raise ValueError(
            f"Motor '{motor_name}' not found. Available: {names[:10]}{' ...' if len(names) > 10 else ''}"
        )
    motor = matches[0]

    battery: Optional[BatteryRecord] = None
    if battery_name and battery_name.lower() != "none":
        bmatches = [b for b in batteries if b.name.lower() == battery_name.lower()]
        if not bmatches:
            bnames = [b.name for b in batteries]
            raise ValueError(f"Battery '{battery_name}' not found. Available: {bnames}")
        b = bmatches[0]
        battery = BatteryRecord(
            name=b.name,
            index=b.index,
            mass=b.mass,
            nominal_voltage=b.nominal_voltage,
            usable_energy_wh=b.usable_energy_wh,
            quantity=battery_quantity,
        )

    return motor, battery
