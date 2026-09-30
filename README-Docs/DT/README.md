# DT (Digital Twin)

6-DOF multirotor sim, ArduPilot JSON link, GUI.

> Interactive map of everything below: [system-architecture.html](../system-architecture.html) (Digital-twin backend ▸ Drone plant). Download and open in a browser.

- Needs ArduPilot SITL built first — see [ArduPilot-SITL-install-guide.md](ArduPilot-SITL-install-guide.md).
- 6-DOF quaternion rigid body. See [drone-plant-readme.md](drone-plant-readme.md) for the physics core (per-tick data flow, equations, RK4 scheme).
- Forces: motor thrust (database curves), gravity, drag, ground contact.
- Dryden gust: configurable `W20_ms`, heading, horizontal/vertical wind, filter step, seeds. See [dryden-turbulence-readme.md](dryden-turbulence-readme.md).
- Ground: spring + damper; lateral friction; rotational damping.
- Run: install; start flight controller in JSON mode; run `python Src/DT/main.py` (or press Launch in the GUI).
- GUI: Setup launch/stop/config; Live Data accel/gyro/altitude/wind/trajectory; Flight Time estimate; Propulsion Model.

## How it runs

`Src/DT/main.py` is one process, one loop. It owns UDP `9002` and never sends first.

```
ArduPilot ──PWM frame (UDP 9002)──► recv_pwm ─► sim.tick(pwm, dt) ─► send_state ──JSON──► ArduPilot
                                                    │
                                                    └─► telemetry (ZMQ 9010, 50 Hz) · CSV row · optional 3D frame
```

Per ArduPilot frame:

1. **Receive** — binary packet: `uint16 magic` (18458 = 16 channels, 29569 = 32), `uint16 frame_rate`, `uint32 frame_count`, `uint16 pwm[16|32]`. Wrong length skipped; lower `frame_count` = ArduPilot restarted → `sim.reset()`; equal = duplicate, skipped.
2. **dt** = `min(1/frame_rate, 1/sim_rate)`. `sim_rate` (400 Hz) is a cap on the step, not a timer — ArduPilot sets the pace, so ArduPilot blocks until the reply arrives.
3. **Step** — `Simulation.tick` (motor PWM → throttle → optional lag → motor curve → forces; ground; gust; drag + gravity; sum; RK4).
4. **Reply** — one newline-framed JSON line to the address of the last frame:
   `{timestamp, imu:{gyro, accel_body}, position (NED), attitude (roll, pitch, yaw rad), velocity (NED)}`.
   `accel_body = accel_bb − DCM·g` (see the review note in the [drone plant doc](drone-plant-readme.md#imu-signal-sent-to-ardupilot)).
5. **Side outputs** — telemetry snapshot (50 Hz of *simulated* time), status message every 10 s wall-clock, optional 3D frame, CSV row.

Only the IMU comes from the twin. GPS, barometer and compass are synthesised by ArduPilot's own SITL models from the position and attitude it is sent; there are no sensor-noise models in this repo. The optional second IMU / magnetometer / barometer JSON fields are **not** sent by the current code.

## Config

`Src/DT/config/startup_config.json` (edited in the GUI Setup tab or by hand; restart to apply):

| Key | Meaning |
|---|---|
| `sim.sim_rate`, `sim.seed` | max step rate (400 Hz); master seed (spawns child seeds; only Dryden uses one) |
| `actuator.enabled`, `motor_tau_s` | first-order motor lag (off by default, τ = 0.07 s) |
| `sitl.udp_port`, `verbose`, `preserve_eeprom` | UDP port (9002); print PWM in status line; `false` → ArduPilot `-w` (wipe EEPROM) |
| `logging.enabled`, `path` | CSV run log, path relative to repo root (default `Results/run.csv`, overwritten each start) |
| `telemetry.enabled`, `endpoint`, `rate_hz` | ZMQ PUB for the GUI (also forced on by env `DT_TELEMETRY_ENABLED=1`, which the GUI launcher sets) |
| `drone_model` | drone JSON name, dashes → underscores (`generic-quad` → `generic_quad.json`) |
| `environment.rho`, `environment.dryden.*` | air density; gust + mean wind (`enabled=false` means **no wind at all**) |
| `initial.*` | start position (`pos_lla` origin, `pos_ned`), body velocity, Euler, rates |
| `viz.enabled`, `fps`, `span_m` | optional matplotlib 3D window (separate process, off by default) |

Ignored keys: `sitl.magic` (frame layout is auto-detected), `sitl.print_every` (status interval fixed at 10 s), `param_file` (the parameter file is chosen in the GUI). With `sim.seed` set, `dryden.seeds` is ignored.

`Src/DT/config/drone_config_variables/*.json` — airframe: `airframe` (mass, CAD inertia in kg·mm², CG and ground offset in mm, `drag_cd`, `surface_area_xyz`, `drag_cd_moment`), `rotors.control[]` (`pwm_ch`, `arm_pos_centroid` mm, `spin_dir` ±1, `motor_name` — must exist in the [database](../database-doc.md)), `pwm`, `ground_contact`. Unused fields: `imu_count`, `battery`, `rotor_aero`, `drag_reference_area`.

`Src/DT/config/drone_sim_param_files/*.parm` — ArduPilot parameters. `dt-ekf-paper.parm` (`AHRS_EKF_TYPE=3`, EKF3) and `dt-ground-truth-paper.parm` (`AHRS_EKF_TYPE=10`, SITL ground truth) differ in that one line.

## Telemetry to the GUI

Twin binds ZMQ PUB `tcp://*:9010` (send high-water-mark 1, non-blocking — a slow or missing subscriber never stalls the physics). Messages:

- `{"type":"telemetry", t, accel, gyro, gps_alt, gps_vn, baro_alt, state_alt, wind_ned, mag_xyz, pos_ned, quat_wxyz}` — 50 Hz of sim time. `accel` is `accel_bb` (gravity included, ≈ 0 in steady hover). The three `*_alt` values are the same true altitude; `mag_xyz` is null.
- `{"type":"status", fps, realtime_pct, t, tick_us, budget_pct, cpu_pct, connected}` — every 10 s of wall time.

The GUI server subscribes, keeps the newest frame, and pushes it to browsers on `/ws/telemetry`.

## Outputs

- `Results/run.csv`: `t, x_n, x_e, x_d, u, v, w, roll, pitch, yaw, p, q, r, realtime_pct` (flushed every row).
- Terminal: `[sitl] Connected to …`, `Controller reset`, `Missed N input frames`, and a periodic `[sitl] fps / realtime / tick µs / cpu` line.

Wind telemetry read-only; edit config, restart to change.
