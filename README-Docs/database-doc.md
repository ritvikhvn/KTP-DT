# Motor/battery database

`Src/database/Motor_battery_database.sqlite`. Simplified — 3 generic motors + 3 batteries, no proprietary part numbers. ([Interactive map](system-architecture.html): *Motor & battery database*.)

Shipped parts — motors: `Motor-A 920KV 10x4.5`, `Motor-B 750KV 18x6`, `AIR2216 KV920 -10*4.5` (the one `generic_quad.json` uses); batteries: `4S LiPo 5000mAh`, `6S LiPo 5000mAh`, `4S Lion 7000mAh`.

## Tables

- `motors`: name, type (`Control` only), mass, prop radius.
- `motor_performance`: 50 synthetic throttle points per motor — voltage, current, power, rpm, torque, thrust, efficiency.
- `motor_raw_data`: same shape, raw (non-interpolated) source points.
- `motor_specs`: Kv, no-load current, peak current, max power, rated voltage, internal resistance.
- `batteries`: chemistry, cell/pack voltages (nominal/max/min/landing), capacity, mass, current limits.
- `battery_processed`: depth-of-discharge, usable energy (Wh/J), max energy density — computed at load, not stored raw.

## Add an entry

- Motor: insert `motors` row, matching `motor_performance` curve, optional `motor_specs`. Keep names generic (no vendor part numbers).
- Battery: insert `batteries` row (cell + pack voltages, capacity, mass, current limits). Processed energy fields computed on read, not written directly.

## Readers

Two independent readers, same file:

- `Src/Design-config-tool/db_api.py` (`get_motors()`, `get_batteries()`, `add_motor()`, `add_battery()`) — dictionaries; used by the [flight-time estimator](Design-config-tool/README.md) and the GUI drop-downs. No cache; each call opens the DB and runs `CREATE TABLE IF NOT EXISTS`. `add_*` are library functions with no GUI route.
- `Src/database/design_config_tool_import/motor_battery_db.py` (`load_databases()`) — NumPy records for the [DT sim](DT/README.md). Cache order: in-memory → `cache/motor_database.pkl` + `battery_database.pkl` → SQLite (reloads if the `.sqlite` is newer than the motor pickle). Pickles are git-ignored, so the first run after a clone builds them. Each motor is resampled onto a 101-point throttle grid (`MotorLookup`) that the physics indexes directly every tick.

`motor_raw_data` and `battery_processed` are created but no runtime path reads them.

## Seed / rebuild

`python Src/database/build_db.py` seeds the file only if it does not exist; `--force` rebuilds it and **discards any parts you added**. Otherwise edit the committed file directly (e.g. DB Browser for SQLite); `motor_type` must be `Control`.
