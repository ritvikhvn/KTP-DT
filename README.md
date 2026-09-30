# KTP-DT

Digital-twin + design-tool package. Output of KTP (Knowledge Transfer Partnership : 10088368) - Digital Twin System

- KTP associate: Ritvik Swaminathan
- Academic supervisors: Dr. Amina Hamoud, Dr. Jun Yao

Standalone digital-twin package — 6-DOF parametric multirotor sim, propulsion model, flight-time estimator, browser GUI.

## Architecture explorer

**[Open the interactive architecture explorer → README-Docs/system-architecture.html](README-Docs/system-architecture.html)**

Simulink-style block diagram of the whole repo and simulator. Click a block with ▸ to drill in (up to four levels, e.g. Digital-twin backend ▸ Drone plant ▸ 6-DOF integrator ▸ RK4 stages), click a wire to see the exact signal, transport, port and rate. Also has "Follow a run" walk-throughs (launch sequence, one physics frame, inside the drone plant), a port map and a file → block repo map.

GitHub shows `.html` as source, so **download the file (or clone) and open it in a browser** — it is a single self-contained page (needs internet only for two CDN assets: the ELK layout library and a mono font).

How it fits together, in one paragraph: the GUI server (`python -m gui`) starts two processes — the digital-twin physics process (`Src/DT/main.py`) and ArduPilot SITL inside WSL. They exchange motor PWM and vehicle state over UDP 9002 in lock-step; the twin also streams telemetry (ZMQ 9010) that the GUI relays to the browser over a WebSocket.

## Docs

| Module | Doc | What's in it |
|---|---|---|
| Architecture | [README-Docs/system-architecture.html](README-Docs/system-architecture.html) | interactive block diagram: GUI, launcher, twin, drone plant, ArduPilot link, DB, design tools |
| DT | [README-Docs/DT/README.md](README-Docs/DT/README.md) | how the twin runs: lock-step loop, JSON link, config, telemetry, outputs |
| Drone plant | [README-Docs/DT/drone-plant-readme.md](README-Docs/DT/drone-plant-readme.md) | per-tick data flow, 6-DOF equations, RK4, motors, ground contact, frames |
| Dryden | [README-Docs/DT/dryden-turbulence-readme.md](README-Docs/DT/dryden-turbulence-readme.md) | gust model, config, seeding, wind conventions |
| Design tool | [README-Docs/Design-config-tool/README.md](README-Docs/Design-config-tool/README.md) | flight-time estimator, motor/battery DB access |
| Propulsion | [README-Docs/Propulsion-model/README.md](README-Docs/Propulsion-model/README.md) | BEMT single rotor + standalone DC motor model |
| Lua scripts | [Lua-scripts-fault-journal-paper/lua-script-readme.md](Lua-scripts-fault-journal-paper/lua-script-readme.md) | ArduPilot fault-injection scripts |
| Database | [README-Docs/database-doc.md](README-Docs/database-doc.md) | motor/battery sqlite schema, readers, cache |
| ArduPilot SITL install | [README-Docs/DT/ArduPilot-SITL-install-guide.md](README-Docs/DT/ArduPilot-SITL-install-guide.md) | clone, build, run the flight controller SITL binary DT connects to |

## Repo layout

| Path | What |
|---|---|
| `gui/` | FastAPI server + launcher + telemetry relay + static single-page GUI |
| `Src/DT/` | digital-twin physics process (`main.py`, `Simulation/`, `config/`) |
| `Src/database/` | motor/battery SQLite file, seed script, loader used by the physics |
| `Src/Design-config-tool/` | flight-time estimator + dictionary-style DB API |
| `Src/Propulsion-model/` | DC motor and BEMT rotor models (`pip install -e`, pyBEMT submodule) |
| `Lua-scripts-fault-journal-paper/` | ArduPilot Lua fault scripts + mission |
| `tests/`, `Src/Propulsion-model/tests/` | pytest suites |
| `setup/` | installer scripts and `requirements.txt` |
| `Results/` | git-ignored: `run.csv`, `gui_state.json` |

Full file → block table: **Repo map** tab of the architecture explorer.

## Setup

Needs ArduPilot SITL built first — see the [SITL install guide](README-Docs/DT/ArduPilot-SITL-install-guide.md) (on Windows, that means WSL Ubuntu; same guide covers it).

```sh
# Windows
setup\setup.bat
# Linux/macOS/WSL
./setup/setup.sh
```

Both create `.venv`, install `setup/requirements.txt`, fetch the pyBEMT submodule and `pip install -e Src/Propulsion-model`. Python 3.10–3.12. `pytest` is not in `requirements.txt` — install it to run the tests.

On Windows, `setup.bat` finishes by asking for two paths (press Enter to keep the default):

- **ArduPilot dir** — where your SITL clone lives in WSL, e.g. `/home/you/ardupilot`. Pasting the Explorer form `\\wsl.localhost\Ubuntu\home\you\ardupilot` also works. Default `~/ardupilot`.
- **Venv activate** — the Python venv ArduPilot was built with, e.g. `~/venv-ardupilot/bin/activate`.

Answers are saved to `Results/gui_state.json` and pre-fill the **ArduPilot dir** / **Venv activate** fields on the GUI Setup tab. Edit them there any time; the value used at Launch is saved as the new default. To change them outside the GUI, re-run `setup\setup.bat` or edit `Results/gui_state.json` (`ardupilot_dir`, `venv_activate`). `setup.sh` does not ask — set them in the GUI or edit that file.

## Run — GUI

Run from repo root.

```sh
# Windows
.venv\Scripts\python.exe -m gui
# Linux/macOS/WSL
.venv/bin/python -m gui
```

Opens `http://127.0.0.1:8080`. Options: `--port 8090`, `--host 0.0.0.0`, `--no-browser`. Refuses to start if the port is taken.

Tabs:

- **Simulator ▸ Setup** — pick a parameter file, click **Detect IP** (Windows + WSL: the field starts at `127.0.0.1`, which is only right for native Linux), check ArduPilot dir / venv, optionally tick **Wipe EEPROM** (`-w`), edit the startup config, **Launch**.
- **Simulator ▸ Live Data** — accel, gyro, altitude, wind and a 3D trail, ~12 s history at 50 Hz. Open it after Launch; frames sent before the tab is first opened are not buffered. The top status bar shows Realtime %, CPU, FPS, sim time and whether ArduPilot is connected.
- **Flight Time** — flight-time estimate from motor, battery and mass budget.
- **Propulsion Model** — DC motor point + throttle sweep; propeller BEMT solve.

Launch needs Windows + WSL (it drives ArduPilot SITL through `wsl.exe`). It starts the physics process first, then opens a Windows Terminal tab (or console) running `sim_vehicle.py` in WSL — that window is the ArduPilot/MAVProxy log. **Stop** kills both. On native Linux/macOS, use the headless CLI below instead.

## Run — headless CLI

Every GUI tab has a CLI equivalent — no browser needed.

**DT sim** (Simulator tab). ArduPilot must already be running in JSON-backend SITL mode and pointed at the configured UDP port.

```sh
.venv/bin/python Src/DT/main.py
```

Reads `Src/DT/config/startup_config.json` for drone model, sim rate, port, logging (paths resolve from the script location, so the working directory does not matter). Start it **before** ArduPilot — the twin never speaks first.

**Flight time** (Flight Time tab) — see [Design-config-tool README](README-Docs/Design-config-tool/README.md#cli).

```sh
python Src/Design-config-tool/flight_time/flight_time_calc.py --motor-name "..." --battery-name "..." --num-motors 4
```

**Propulsion model** (Propulsion Model tab, both motor + propeller) — see [Propulsion-model README](README-Docs/Propulsion-model/README.md#cli).

```sh
python Src/Propulsion-model/aerodynamic_model/motor.py --kv 920 --v-batt 14.8 --throttle 0.7
python Src/Propulsion-model/analysis_scripts/single_rotor_analysis.py --diameter-in 10 --pitch-in 4.5 --rpm 6000
```

All four take `--help` for their full flag list.

## Ports

| Port | Proto | Who → who |
|---|---|---|
| 8080 | HTTP / WebSocket | browser ↔ GUI server (`--port` to change) |
| 9002 | UDP | ArduPilot ↔ twin (PWM frames in, JSON state out) |
| 9010 | ZMQ (tcp) | twin publishes telemetry, GUI server subscribes |
| 14551 | UDP | MAVProxy → any ground-control software on the host (optional) |

Full list with sources: **Port map** tab of the architecture explorer.

## Outputs

- `Results/run.csv` — one row per physics frame (`t`, position NED, body velocity, Euler, body rates, realtime %). Overwritten on every start. Disable with `logging.enabled=false`.
- `Results/gui_state.json` — saved host IP, ArduPilot dir, venv activate.
- Optional matplotlib 3D window: `viz.enabled=true` in `startup_config.json`.

## Tests

```sh
pip install pytest
pytest tests Src/Propulsion-model/tests
```

Covers motor lag / PWM mapping and the propulsion models. The SITL link, integrator, GUI server, launcher and flight-time calculator have no tests yet.

## Known review items

Recorded while writing the architecture explorer (code wins over docs):

- IMU accel sent to ArduPilot is `accel_bb − DCM·g`, where `accel_bb = F/m + v_body×ω`. A textbook accelerometer has no `ω×v` term — confirm this is intended.
- `wind_heading_deg` is documented as the direction wind blows *from*, but the mean-wind vector is built pointing *toward* it.
- The launcher's "UDP port 9002 already in use" check binds a TCP socket, so a stale UDP listener is not detected.
- `startup_config.json` keys `sitl.magic`, `sitl.print_every`, `param_file`, and `environment.dryden.seeds` (once `sim.seed` is set) are ignored.
