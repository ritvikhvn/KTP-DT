"""Build compact generic runtime database from synthetic rows."""
from __future__ import annotations

import math
import sqlite3
from pathlib import Path

DB_PATH = Path(__file__).resolve().parent / "Motor_battery_database.sqlite"

SCHEMA = """
CREATE TABLE batteries (battery_name TEXT PRIMARY KEY, chemistry TEXT, cells_series INTEGER,
 cells_parallel INTEGER, charge_ah REAL, nominal_voltage_cell REAL, max_voltage_cell REAL,
 min_voltage_cell REAL, landing_voltage_cell REAL, nominal_voltage_battery REAL,
 max_voltage_battery REAL, min_voltage_battery REAL, landing_voltage_battery REAL,
 total_mass_kg REAL, continuous_discharge_current_a REAL, peak_current_a REAL);
CREATE TABLE motors (motor_id INTEGER PRIMARY KEY AUTOINCREMENT, motor_name TEXT NOT NULL,
 motor_type TEXT NOT NULL CHECK(motor_type IN ('Control')), mass_kg REAL, prop_radius_m REAL);
CREATE TABLE motor_performance (motor_id INTEGER REFERENCES motors(motor_id) ON DELETE CASCADE,
 throttle_normalized REAL, voltage_v REAL, current_a REAL, power_w REAL, rpm REAL,
 torque_nm REAL, thrust_g REAL, efficiency_g_per_w REAL,
 PRIMARY KEY(motor_id, throttle_normalized));
CREATE TABLE motor_raw_data (motor_id INTEGER REFERENCES motors(motor_id) ON DELETE CASCADE,
 throttle_pct REAL, voltage_v REAL, current_a REAL, power_w REAL, rpm REAL, torque_nm REAL,
 thrust_g REAL, efficiency_g_per_w REAL, PRIMARY KEY(motor_id, throttle_pct));
CREATE TABLE motor_specs (motor_id INTEGER PRIMARY KEY REFERENCES motors(motor_id) ON DELETE CASCADE,
 kv_rating REAL, no_load_current_a REAL, peak_current_a REAL, max_power_w REAL,
 rated_voltage_v REAL, internal_resistance_mohm REAL);
CREATE TABLE battery_processed (battery_name TEXT PRIMARY KEY REFERENCES batteries(battery_name),
 depth_of_discharge REAL, total_usable_energy_wh REAL, total_usable_energy_j REAL,
 max_energy_density_wh_kg REAL);
"""

MOTORS = [
    ("Motor-A 920KV 10x4.5", "Control", 0.085, 0.127),
    ("Motor-B 750KV 18x6", "Control", 0.110, 0.229),
    ("AIR2216 KV920 -10*4.5", "Control", 0.064, 0.127000254000508),
]

BATTERIES = [
    ("4S LiPo 5000mAh", "LiPo", 4, 1, 5.0, 3.7, 4.2, 3.3, 3.5, 0.55, 80.0, 120.0),
    ("6S LiPo 5000mAh", "LiPo", 6, 1, 5.0, 3.7, 4.2, 3.3, 3.5, 0.78, 80.0, 120.0),
    ("4S Lion 7000mAh", "Lion", 4, 2, 7.0, 3.7, 4.2, 2.8, 3.3, 0.42, 40.0, 70.0),
]

# Measured throttle-sweep curve for AIR2216 KV920. Columns match motor_performance:
# (throttle_normalized, voltage_v, current_a, power_w, rpm, torque_nm, thrust_g, efficiency_g_per_w)
AIR2216_PERF = [
    (0.02, 16.0, 0.0, 0.0, 927.513665, 0.0, 0.0, 0.0),
    (0.04, 16.0, 0.0, 0.0, 1153.378655, 0.0, 0.0, 0.0),
    (0.06, 16.0, 0.0, 0.0, 1377.522379, 0.0, 0.0, 0.0),
    (0.08, 16.0, 0.0, 0.0, 1599.944835, 0.0, 0.0, 0.0),
    (0.1, 16.0, 0.0, 0.0, 1820.646025, 0.0, 0.0, 0.0),
    (0.12, 16.0, 0.0, 0.0, 2039.625947, 0.0, 0.0, 0.0),
    (0.14, 16.0, 0.0, 0.0, 2256.884602, 0.001586, 0.0, 0.0),
    (0.16, 16.0, 0.199307, 2.215947, 2472.421991, 0.004746, 0.0, 0.0),
    (0.18, 16.0, 0.404635, 5.705924, 2686.238112, 0.007961, 24.53501, 4.299919),
    (0.2, 16.0, 0.593089, 8.901432, 2898.332967, 0.011231, 49.598901, 5.572014),
    (0.22, 16.0, 0.768747, 11.869396, 3108.706555, 0.014556, 75.055294, 6.32343),
    (0.24, 16.0, 0.935468, 14.673168, 3317.358875, 0.017937, 100.904189, 6.876783),
    (0.26, 16.0, 1.096892, 17.372523, 3524.289929, 0.021373, 127.145585, 7.318775),
    (0.28, 16.0, 1.256441, 20.023666, 3729.499716, 0.024865, 153.779483, 7.679887),
    (0.3, 16.0, 1.417317, 22.679223, 3932.988235, 0.028412, 180.805882, 7.972314),
    (0.32, 16.0, 1.582503, 25.388248, 4134.755488, 0.032014, 208.224783, 8.201621),
    (0.34, 16.0, 1.754763, 28.196221, 4334.801474, 0.035671, 236.036186, 8.371199),
    (0.36, 16.0, 1.936643, 31.145047, 4533.126193, 0.039384, 264.24009, 8.484177),
    (0.38, 16.0, 2.13047, 34.273057, 4729.729644, 0.043153, 292.836496, 8.544219),
    (0.4, 16.0, 2.33835, 37.615007, 4924.611829, 0.046976, 321.825404, 8.555772),
    (0.42, 16.0, 2.562172, 41.202079, 5117.772747, 0.050855, 351.206813, 8.524007),
    (0.44, 16.0, 2.803606, 45.061881, 5309.212398, 0.054789, 380.980724, 8.454612),
    (0.46, 16.0, 3.064102, 49.218447, 5498.930782, 0.058779, 411.147136, 8.353517),
    (0.48, 16.0, 3.344891, 53.692235, 5686.927899, 0.062824, 441.70605, 8.226628),
    (0.5, 16.0, 3.646987, 58.500132, 5873.203749, 0.066924, 472.657466, 8.079597),
    (0.52, 16.0, 3.971181, 63.655446, 6057.758332, 0.071079, 504.001383, 7.917648),
    (0.54, 16.0, 4.31805, 69.167915, 6240.591648, 0.07529, 535.737802, 7.745467),
    (0.56, 16.0, 4.687949, 75.043699, 6421.703697, 0.079556, 567.866723, 7.567147),
    (0.58, 16.0, 5.081013, 81.285388, 6601.09448, 0.083878, 600.388145, 7.386176),
    (0.6, 16.0, 5.497161, 87.891993, 6778.763995, 0.088255, 633.302069, 7.205458),
    (0.62, 16.0, 5.936092, 94.858955, 6954.712243, 0.092687, 666.608494, 7.027365),
    (0.64, 16.0, 6.397285, 102.178137, 7128.939224, 0.097174, 700.307421, 6.853789),
    (0.66, 16.0, 6.880001, 109.837829, 7301.444939, 0.101717, 734.398849, 6.686211),
    (0.68, 16.0, 7.383281, 117.822749, 7472.229386, 0.106315, 768.88278, 6.525758),
    (0.7, 16.0, 7.905948, 126.114036, 7641.292566, 0.110969, 803.759211, 6.373273),
    (0.72, 16.0, 8.446606, 134.689259, 7808.63448, 0.115678, 839.028145, 6.229362),
    (0.74, 16.0, 9.003639, 143.522411, 7974.255126, 0.120442, 874.68958, 6.094446),
    (0.76, 16.0, 9.575214, 152.58391, 8138.154505, 0.125262, 910.743516, 5.968804),
    (0.78, 16.0, 10.159278, 161.8406, 8300.332618, 0.130136, 947.189955, 5.85261),
    (0.8, 16.0, 10.753557, 171.255752, 8460.789463, 0.135067, 984.028895, 5.745961),
    (0.82, 16.0, 11.355561, 180.789061, 8619.525042, 0.140052, 1021.260336, 5.648906),
    (0.84, 16.0, 11.96258, 190.396648, 8776.539354, 0.145093, 1058.884279, 5.561465),
    (0.86, 16.0, 12.571684, 200.031061, 8931.832398, 0.150189, 1096.900724, 5.483652),
    (0.88, 16.0, 13.179726, 209.641273, 9085.404176, 0.155341, 1135.30967, 5.415487),
    (0.9, 16.0, 13.783338, 219.172681, 9237.254686, 0.160548, 1174.111118, 5.357014),
    (0.92, 16.0, 14.378934, 228.56711, 9387.38393, 0.16581, 1213.305068, 5.30831),
    (0.94, 16.0, 14.96271, 237.762809, 9535.791907, 0.171127, 1252.891519, 5.269502),
    (0.96, 16.0, 15.530641, 246.694454, 9682.478617, 0.1765, 1292.870472, 5.240776),
    (0.98, 16.0, 16.078484, 255.293146, 9827.444059, 0.181928, 1333.241926, 5.222396),
    (1.0, 16.0, 16.601778, 263.486412, 9970.688235, 0.187412, 1374.005882, 5.214712),
]


def performance(index: int):
    thrust_max = (1750.0, 2350.0)[index]
    rpm_max = (10500.0, 8800.0)[index]
    current_max = (42.0, 48.0)[index]
    for i in range(1, 51):
        t = i / 50.0
        thrust = thrust_max * t * t
        current = 0.8 + current_max * t
        rpm = rpm_max * t
        power = 14.8 * current
        torque = power / (rpm * 2.0 * math.pi / 60.0) if rpm else 0.0
        yield (t, 14.8, current, power, rpm, torque, thrust, thrust / power if power else 0.0)


def build(path: Path = DB_PATH, force: bool = False):
    """Seed the sqlite DB with the default motors/batteries.

    Only runs if `path` doesn't exist yet (or `force=True`). The sqlite file is the
    persistent store from here on — open it directly (e.g. DB Browser for SQLite) to
    add or edit motors/batteries. The loader (design_config_tool_import/motor_battery_db.py)
    detects on-disk changes via mtime and refreshes its cache automatically.
    """
    if path.exists() and not force:
        print(f"{path} already exists, skipping seed (pass force=True to overwrite)")
        return
    path.unlink(missing_ok=True)
    with sqlite3.connect(path) as conn:
        conn.executescript(SCHEMA)
        for i, (name, motor_type, mass, radius) in enumerate(MOTORS):
            cur = conn.execute("INSERT INTO motors(motor_name,motor_type,mass_kg,prop_radius_m) VALUES(?,?,?,?)", (name, motor_type, mass, radius))
            motor_id = cur.lastrowid
            curve = AIR2216_PERF if name == "AIR2216 KV920 -10*4.5" else list(performance(i))
            conn.executemany("INSERT INTO motor_performance VALUES(?,?,?,?,?,?,?,?,?)", [(motor_id, *row) for row in curve])
            conn.execute("INSERT INTO motor_specs VALUES(?,?,?,?,?,?,?)", (motor_id, 920.0, 0.8, 50.0, 700.0, 14.8, 115.0))
        for name, chemistry, series, parallel, charge, cell_nom, cell_max, cell_min, cell_land, mass, continuous, peak in BATTERIES:
            conn.execute("INSERT INTO batteries VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", (name, chemistry, series, parallel, charge, cell_nom, cell_max, cell_min, cell_land, series*cell_nom, series*cell_max, series*cell_min, series*cell_land, mass, continuous, peak))


if __name__ == "__main__":
    import sys
    build(force="--force" in sys.argv)
    print(f"built {DB_PATH}")
