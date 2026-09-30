# Design config tool

Flight-time estimator from a motor/battery database + drone config. ([Interactive map](../system-architecture.html): *Design-config tool ▸ calculate_flight_time()*, seven steps.)

- DB: `Src/database/Motor_battery_database.sqlite`.
- Curves: 50 synthetic points per generic motor.
- Stage 1: capability check at `required_twr`.
- Stage 2: operating point at `flight_twr`.
- Energy: `usable_Wh × 60 / total_power`.
- Config keys: mass, payload, motor/battery indexes or names, counts, ratio, T/W values, bank angle, avionics power, gravity.
- Add motor: insert generic row + curve + specs; keep names generic.
- Add battery: insert cell, capacity, voltage, mass, current limits; processed energy computed at load.
- GUI: **Flight Time** tab → `POST /api/flight-time`. The GUI posts every value itself; it does **not** read `drone_config.json` (CLI base file only). `max_empty_weight` has no default — omitting it gives a 400.
- Motor/battery lists come from `db_api.get_motors()` / `get_batteries()` (see [database-doc](../database-doc.md)). `--motor-idx` counts in `motor_id` order, `--battery-idx` in alphabetical battery-name order.
- `drone_config.json` keys `g` and `voltage_check` are not read.
- Errors: `empty weight limit exceeded`, `motor cannot meet required thrust ratio`.

## CLI

Runs standalone, same math as the GUI's Flight Time tab. Base values come from `drone_config.json`; any flag overrides just that field.

```sh
# from Src/Design-config-tool
python flight_time/flight_time_calc.py --motor-name "Motor-A 920KV 10x4.5" --battery-name "4S LiPo 5000mAh" --num-motors 4
python flight_time/flight_time_calc.py --motor-idx 2 --battery-idx 0 --required-twr 2 --flight-twr 1
python flight_time/flight_time_calc.py --help   # full flag list (structural-mass, payload-mass, bank-angle, avionics-power, ...)
```

## Theory

MTOW:

```
MTOW = structural_mass + payload_mass + n_motors*motor_mass + n_batteries*battery_mass
```

Thrust-to-weight required per motor, corrected for bank angle:

```
T/W_total = required_twr / cos(bank_angle)
T/W_per_motor = T/W_total / n_motors
```

Reject config if no throttle point on the motor curve reaches `T/W_per_motor`.

Operating point: first curve row where `thrust/MTOW >= flight_twr_per_motor` (bank-corrected same way) — its `power_w` becomes the per-motor cruise power.

Total power:

```
P_total = avionics_power + n_motors * P_motor_at_operating_point
```

Usable battery energy (depth-of-discharge from landing-voltage cutoff):

```
DoD = (V_max - V_land) / (V_max - V_min)
Wh_usable = charge_Ah * V_nominal * DoD
```

Flight time:

```
t_min = n_batteries * Wh_usable * 60 / P_total
```
