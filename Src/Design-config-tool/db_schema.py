"""Small SQLite schema for generic motor and battery records."""
from __future__ import annotations

import sqlite3
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
DB_PATH = BASE / "database" / "Motor_battery_database.sqlite"


def create_database(db_path: str | Path = DB_PATH) -> None:
    """Create all runtime tables if missing."""
    path = Path(db_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(path) as conn:
        conn.executescript(
            """
            PRAGMA foreign_keys = ON;
            CREATE TABLE IF NOT EXISTS batteries (
                battery_name TEXT PRIMARY KEY,
                chemistry TEXT,
                cells_series INTEGER,
                cells_parallel INTEGER,
                charge_ah REAL,
                nominal_voltage_cell REAL,
                max_voltage_cell REAL,
                min_voltage_cell REAL,
                landing_voltage_cell REAL,
                nominal_voltage_battery REAL,
                max_voltage_battery REAL,
                min_voltage_battery REAL,
                landing_voltage_battery REAL,
                total_mass_kg REAL,
                continuous_discharge_current_a REAL,
                peak_current_a REAL
            );
            CREATE TABLE IF NOT EXISTS motors (
                motor_id INTEGER PRIMARY KEY AUTOINCREMENT,
                motor_name TEXT NOT NULL,
                motor_type TEXT NOT NULL CHECK(motor_type IN ('Control')),
                mass_kg REAL,
                prop_radius_m REAL
            );
            CREATE TABLE IF NOT EXISTS motor_performance (
                motor_id INTEGER NOT NULL REFERENCES motors(motor_id) ON DELETE CASCADE,
                throttle_normalized REAL NOT NULL,
                voltage_v REAL,
                current_a REAL,
                power_w REAL,
                rpm REAL,
                torque_nm REAL,
                thrust_g REAL,
                efficiency_g_per_w REAL,
                PRIMARY KEY (motor_id, throttle_normalized)
            );
            CREATE TABLE IF NOT EXISTS motor_raw_data (
                motor_id INTEGER NOT NULL REFERENCES motors(motor_id) ON DELETE CASCADE,
                throttle_pct REAL NOT NULL,
                voltage_v REAL,
                current_a REAL,
                power_w REAL,
                rpm REAL,
                torque_nm REAL,
                thrust_g REAL,
                efficiency_g_per_w REAL,
                PRIMARY KEY (motor_id, throttle_pct)
            );
            CREATE TABLE IF NOT EXISTS motor_specs (
                motor_id INTEGER PRIMARY KEY REFERENCES motors(motor_id) ON DELETE CASCADE,
                kv_rating REAL,
                no_load_current_a REAL,
                peak_current_a REAL,
                max_power_w REAL,
                rated_voltage_v REAL,
                internal_resistance_mohm REAL
            );
            CREATE TABLE IF NOT EXISTS battery_processed (
                battery_name TEXT PRIMARY KEY REFERENCES batteries(battery_name) ON DELETE CASCADE,
                depth_of_discharge REAL,
                total_usable_energy_wh REAL,
                total_usable_energy_j REAL,
                max_energy_density_wh_kg REAL
            );
            """
        )


def insert_motor(conn, motor_name, motor_type, mass_kg, prop_radius_m, performance_rows, specs=None):
    cur = conn.execute(
        "INSERT INTO motors (motor_name,motor_type,mass_kg,prop_radius_m) VALUES (?,?,?,?)",
        (motor_name, motor_type, float(mass_kg), float(prop_radius_m)),
    )
    motor_id = cur.lastrowid
    conn.executemany(
        "INSERT INTO motor_performance VALUES (?,?,?,?,?,?,?,?,?)",
        [(motor_id, *row) for row in performance_rows],
    )
    if specs:
        conn.execute(
            "INSERT INTO motor_specs VALUES (?,?,?,?,?,?,?)",
            (motor_id, specs["kv_rating"], specs["no_load_current_a"], specs["peak_current_a"],
             specs["max_power_w"], specs["rated_voltage_v"], specs["internal_resistance_mohm"]),
        )
    return motor_id


def insert_motor_specs(conn, motor_id, specs):
    conn.execute(
        "INSERT OR REPLACE INTO motor_specs VALUES (?,?,?,?,?,?,?)",
        (motor_id, specs["kv_rating"], specs["no_load_current_a"], specs["peak_current_a"],
         specs["max_power_w"], specs["rated_voltage_v"], specs["internal_resistance_mohm"]),
    )


def insert_battery(conn, fields: dict):
    columns = [
        "battery_name", "chemistry", "cells_series", "cells_parallel", "charge_ah",
        "nominal_voltage_cell", "max_voltage_cell", "min_voltage_cell", "landing_voltage_cell",
        "nominal_voltage_battery", "max_voltage_battery", "min_voltage_battery",
        "landing_voltage_battery", "total_mass_kg", "continuous_discharge_current_a", "peak_current_a",
    ]
    conn.execute(
        f"INSERT INTO batteries ({','.join(columns)}) VALUES ({','.join('?' for _ in columns)})",
        [fields.get(column) for column in columns],
    )


def get_motor_names(db_path: str | Path = DB_PATH) -> list[str]:
    create_database(db_path)
    with sqlite3.connect(db_path) as conn:
        return [row[0] for row in conn.execute("SELECT motor_name FROM motors ORDER BY motor_name")]


def get_battery_names(db_path: str | Path = DB_PATH) -> list[str]:
    create_database(db_path)
    with sqlite3.connect(db_path) as conn:
        return [row[0] for row in conn.execute("SELECT battery_name FROM batteries ORDER BY battery_name")]
