"""Read and write helpers for the compact design database."""
from __future__ import annotations

import sqlite3
from pathlib import Path

try:
    from .db_schema import DB_PATH, create_database, insert_battery, insert_motor
except ImportError:
    from db_schema import DB_PATH, create_database, insert_battery, insert_motor

PERF_COLUMNS = (
    "throttle_normalized", "voltage_v", "current_a", "power_w", "rpm",
    "torque_nm", "thrust_g", "efficiency_g_per_w",
)
SPECS_COLUMNS = (
    "kv_rating", "no_load_current_a", "peak_current_a", "max_power_w",
    "rated_voltage_v", "internal_resistance_mohm",
)
VALID_MOTOR_TYPES = {"Control"}


def open_conn(db_path: str | Path = DB_PATH):
    create_database(db_path)
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA foreign_keys = ON")
    return conn


def get_motors(db_path: str | Path = DB_PATH) -> list[dict]:
    with open_conn(db_path) as conn:
        rows = conn.execute(
            """SELECT m.motor_id,m.motor_name,m.motor_type,m.mass_kg,m.prop_radius_m,
                      p.throttle_normalized,p.voltage_v,p.current_a,p.power_w,p.rpm,
                      p.torque_nm,p.thrust_g,p.efficiency_g_per_w
               FROM motors m LEFT JOIN motor_performance p ON p.motor_id=m.motor_id
               ORDER BY m.motor_id,p.throttle_normalized"""
        ).fetchall()
        specs = {
            row[0]: dict(zip(SPECS_COLUMNS, row[1:]))
            for row in conn.execute("SELECT motor_id," + ",".join(SPECS_COLUMNS) + " FROM motor_specs")
        }
    result = {}
    for row in rows:
        result.setdefault(row[0], {
            "motor_id": row[0], "motor_name": row[1], "motor_type": row[2],
            "mass_kg": row[3], "prop_radius_m": row[4], "performance": [],
            "specs": specs.get(row[0]),
        })
        if row[5] is not None:
            result[row[0]]["performance"].append(dict(zip(PERF_COLUMNS, row[5:])))
    return list(result.values())


def get_batteries(db_path: str | Path = DB_PATH) -> list[dict]:
    with open_conn(db_path) as conn:
        rows = conn.execute("SELECT * FROM batteries ORDER BY battery_name").fetchall()
        columns = [item[1] for item in conn.execute("PRAGMA table_info(batteries)")]
    result = []
    for row in rows:
        item = dict(zip(columns, row))
        vmax, vmin = item.get("max_voltage_battery") or 0, item.get("min_voltage_battery") or 0
        vland, vnom = item.get("landing_voltage_battery") or 0, item.get("nominal_voltage_battery") or 0
        charge, mass = item.get("charge_ah") or 0, item.get("total_mass_kg") or 0
        dod = (vmax - vland) / (vmax - vmin) if vmax > vmin else 0.0
        usable = charge * vnom * dod
        item.update(depth_of_discharge=dod, total_usable_energy_wh=usable,
                    total_usable_energy_j=usable * 3600.0,
                    max_energy_density_wh_kg=usable / mass if mass else 0.0)
        result.append(item)
    return result


def get_battery_processed(battery_name, db_path: str | Path = DB_PATH):
    with open_conn(db_path) as conn:
        row = conn.execute("SELECT * FROM battery_processed WHERE battery_name=?", (battery_name,)).fetchone()
        if row is None:
            return None
        cols = [item[1] for item in conn.execute("PRAGMA table_info(battery_processed)")]
    return dict(zip(cols, row))


def motor_exists(name, db_path: str | Path = DB_PATH):
    with open_conn(db_path) as conn:
        return conn.execute("SELECT 1 FROM motors WHERE motor_name=?", (name,)).fetchone() is not None


def battery_exists(name, db_path: str | Path = DB_PATH):
    with open_conn(db_path) as conn:
        return conn.execute("SELECT 1 FROM batteries WHERE battery_name=?", (name,)).fetchone() is not None


def validate_motor_payload(payload):
    errors = []
    if not (payload.get("motor_name") or "").strip():
        errors.append("motor_name is required")
    if payload.get("motor_type") not in VALID_MOTOR_TYPES:
        errors.append("motor_type is invalid")
    for key in ("mass_kg", "prop_radius_m"):
        try:
            if float(payload[key]) <= 0: errors.append(f"{key} must be positive")
        except (KeyError, TypeError, ValueError):
            errors.append(f"{key} must be numeric")
    if len(payload.get("performance_rows", [])) < 2:
        errors.append("at least two performance rows required")
    return errors


def validate_battery_payload(payload):
    errors = []
    for key in ("battery_name", "chemistry"):
        if not (payload.get(key) or "").strip(): errors.append(f"{key} is required")
    for key in ("cells_series", "cells_parallel", "charge_ah", "total_mass_kg"):
        try:
            if float(payload[key]) <= 0: errors.append(f"{key} must be positive")
        except (KeyError, TypeError, ValueError):
            errors.append(f"{key} must be numeric")
    return errors


def add_motor(payload, db_path: str | Path = DB_PATH):
    errors = validate_motor_payload(payload)
    if errors: raise ValueError(errors)
    name = payload["motor_name"].strip()
    if motor_exists(name, db_path): raise ValueError([f"Motor '{name}' already exists"])
    conn = open_conn(db_path)
    try:
        rows = [tuple(row.get(key) for key in PERF_COLUMNS) for row in payload["performance_rows"]]
        motor_id = insert_motor(conn, name, payload["motor_type"], payload["mass_kg"], payload["prop_radius_m"], rows, payload.get("specs"))
        conn.commit()
        return motor_id
    finally:
        conn.close()


def add_battery(payload, db_path: str | Path = DB_PATH):
    errors = validate_battery_payload(payload)
    if errors: raise ValueError(errors)
    name = payload["battery_name"].strip()
    if battery_exists(name, db_path): raise ValueError([f"Battery '{name}' already exists"])
    series = int(payload["cells_series"])
    fields = dict(payload)
    fields.update(battery_name=name, cells_series=series, cells_parallel=int(payload["cells_parallel"]))
    for key in ("nominal_voltage_cell", "max_voltage_cell", "min_voltage_cell", "landing_voltage_cell"):
        fields[key.replace("_cell", "_battery")] = series * float(payload[key])
    conn = open_conn(db_path)
    try:
        insert_battery(conn, fields); conn.commit()
    finally:
        conn.close()
