# Lua scripts

Static ArduPilot fault-injection scripts (run on the flight-controller SITL binary, not the digital twin).

- Place ArduPilot scripts here. ([Interactive map](../README-Docs/system-architecture.html): *ArduPilot SITL ▸ Lua scripting*.)
- Scripts may drive stock `SIM_ACC1_*` and `SIM_VIB_*` parameters — ArduPilot's own SITL accel-bias/vibration-injection knobs, set from Lua at runtime to simulate sensor faults.
- Copy scripts into `APM/scripts/`.
- Enable scripting with `SCR_ENABLE=1`.

## Files

- `Lua-scripts-journal-paper/imu_bias_z_<±0.5|1.0|2.0|5.0>.lua` — one severity per run: sets `SIM_ACC1_BIAS_Z` once mission waypoint `START_WP` (168 in these scripts) is reached and holds it. `m` in the file name means negative (`imu_bias_z_m1p0` = −1.0 m/s²). No script = nominal baseline.
- `imu_param_sweep.lua` — sequential sweep (accel bias → accel noise → vibration frequency → noise + vibration), each held then reverted; starts at waypoint 2.
- `journal-paper-heart-mission.waypoints` — the mission the scripts trigger on (load it in a ground station).

## Notes

- The faults act on ArduPilot's own SITL sensor layer (`SIM_*` parameters), **not** in the digital twin, and take effect only while motors are running.
- Scripts poll every 1 s until armed and the waypoint index is reached, so they do nothing on the ground.
