"""Flight-time predictor using compact motor curves and battery data."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import numpy as np

_DESIGN_DIR = Path(__file__).resolve().parents[1]
if str(_DESIGN_DIR) not in sys.path:
    sys.path.insert(0, str(_DESIGN_DIR))

try:
    from ..db_api import get_batteries, get_motors
except ImportError:
    from db_api import get_batteries, get_motors

CONFIG_PATH = Path(__file__).resolve().parents[1] / "drone_config.json"


def _pick(items, key, value):
    if isinstance(value, int):
        return items[value]
    value = str(value).lower()
    return next(item for item in items if str(item[key]).lower() == value)


def _curve(motor):
    rows = motor["performance"]
    return {key: np.asarray([row[key] for row in rows], dtype=float) for key in ("throttle_normalized", "voltage_v", "current_a", "power_w", "rpm", "thrust_g", "torque_nm")}


def calculate_flight_time(design_params, verbose=True):
    motors, batteries = get_motors(), get_batteries()
    motor = _pick(motors, "motor_name", design_params.get("motor_name", design_params.get("Motor_idx", 0)))
    battery = _pick(batteries, "battery_name", design_params.get("battery_name", design_params.get("Battery_idx", 0)))
    n = int(design_params.get("num_motors", 4)); nbat = int(design_params.get("num_batteries", 1))
    structural = float(design_params.get("structural_mass", 0.6)); payload = float(design_params.get("payload_mass", 0.1))
    mmass = float(motor["mass_kg"])
    MTOW = structural + payload + n * mmass + nbat * float(battery["total_mass_kg"])
    empty = MTOW - payload
    if empty > float(design_params["max_empty_weight"]):
        raise ValueError("empty weight limit exceeded")
    bank = float(design_params.get("bank_angle", 0))
    req = float(design_params.get("required_twr", 2.0)); flight = float(design_params.get("flight_twr", req))
    angle = np.cos(np.deg2rad(bank)); total_req = req / angle
    curve = _curve(motor)
    twr = curve["thrust_g"] / (MTOW * 1000.0)
    req_per_motor = total_req / n
    if twr.max() < req_per_motor: raise ValueError("motor cannot meet required thrust ratio")
    req_per_motor_f = (flight / angle) / n
    i = int(np.flatnonzero(twr >= req_per_motor_f)[0]); power = curve["power_w"][i]
    total_power = float(design_params.get("avionics_power", 6.0)) + n * power
    vmax, vmin, vland = battery["max_voltage_battery"], battery["min_voltage_battery"], battery["landing_voltage_battery"]
    dod = (vmax - vland) / (vmax - vmin) if vmax > vmin else 0.0
    usable = float(battery["charge_ah"]) * float(battery["nominal_voltage_battery"]) * dod
    energy = nbat * usable
    max_raw = (n * curve["thrust_g"].max()) / 1000.0
    result = {"flight_time_min": energy * 60.0 / total_power if total_power else 0.0, "empty_weight_kg": empty, "payload_ratio": payload / empty if empty else 0.0, "required_twr": req, "flight_twr": flight, "MTOW_kg": MTOW, "motor_name": motor["motor_name"], "battery_name": battery["battery_name"], "num_batteries": nbat, "num_motors": n, "motor_throttle": curve["throttle_normalized"][i], "motor_mass": mmass, "battery_mass": battery["total_mass_kg"], "battery_usable_energy_wh": usable, "total_battery_energy_wh": energy, "depth_of_discharge": dod, "max_thrust_raw_kg": max_raw, "total_power_w": total_power}
    return {k: (float(v) if isinstance(v, np.floating) else v) for k, v in result.items()}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Flight-time estimator")
    parser.add_argument("--motor-name", help="motor name (overrides --motor-idx)")
    parser.add_argument("--battery-name", help="battery name (overrides --battery-idx)")
    parser.add_argument("--motor-idx", type=int, help="motor index in the database")
    parser.add_argument("--battery-idx", type=int, help="battery index in the database")
    parser.add_argument("--num-motors", type=int)
    parser.add_argument("--num-batteries", type=int)
    parser.add_argument("--structural-mass", type=float, help="[kg]")
    parser.add_argument("--payload-mass", type=float, help="[kg]")
    parser.add_argument("--max-empty-weight", type=float, help="[kg]")
    parser.add_argument("--bank-angle", type=float, help="[deg]")
    parser.add_argument("--required-twr", type=float)
    parser.add_argument("--flight-twr", type=float)
    parser.add_argument("--avionics-power", type=float, help="[W]")
    args = parser.parse_args()

    params = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    overrides = {
        "motor_name": args.motor_name, "battery_name": args.battery_name,
        "Motor_idx": args.motor_idx, "Battery_idx": args.battery_idx,
        "num_motors": args.num_motors, "num_batteries": args.num_batteries,
        "structural_mass": args.structural_mass, "payload_mass": args.payload_mass,
        "max_empty_weight": args.max_empty_weight, "bank_angle": args.bank_angle,
        "required_twr": args.required_twr, "flight_twr": args.flight_twr,
        "avionics_power": args.avionics_power,
    }
    params.update({key: value for key, value in overrides.items() if value is not None})

    result = calculate_flight_time(params, verbose=False)
    assert np.isfinite(result["flight_time_min"]) and result["flight_time_min"] > 0
    print(f"flight time: {result['flight_time_min']:.2f} min")
    for key, value in result.items():
        print(f"  {key}: {value}")
