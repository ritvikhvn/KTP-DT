# ArduPilot SITL install guide

DT talks to a real ArduPilot flight-controller binary running in SITL (Software-In-The-Loop), over ArduPilot's JSON physics-backend protocol (UDP). This guide builds that binary from scratch. ArduPilot itself is open source (github.com/ArduPilot/ardupilot) — nothing here is HVN/UWE-specific.

Tested against **Copter-4.6**. Other Copter-4.x branches likely work; anything much older/newer may need param-file or protocol tweaks.

## 1. Platform

- **Linux / macOS**: build natively, skip to [step 2](#2-clone-ardupilot).
- **Windows**: ArduPilot's build tooling is Linux-only. Install WSL2 + Ubuntu first:

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if prompted, then open the "Ubuntu" app once to finish first-run setup (creates your WSL Linux user). Everything from here runs **inside that WSL/Ubuntu shell**, not PowerShell.

## 2. Clone ArduPilot

```sh
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-4.6
git submodule update --init --recursive
```

The GUI defaults to `~/ardupilot`. Cloned it somewhere else? Tell `setup\setup.bat` when it asks (or set **ArduPilot dir** in the GUI's Setup tab) instead of moving it — it's saved to `Results/gui_state.json` and reused on future launches. The Explorer form `\\wsl.localhost\Ubuntu\home\you\ardupilot` is accepted and converted to `/home/you/ardupilot`. A bad path is caught before launch with a clear error ("ArduPilot not found at … — check the path in Setup"), instead of a WSL terminal that silently fails.

## 3. Install build prerequisites

```sh
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

This installs the ArduPilot toolchain (gcc-arm-none-eabi, ccache, python build deps, etc.) and creates a Python virtualenv.

## 4. Python environment

The GUI defaults to an activatable venv at `~/venv-ardupilot/bin/activate`. If `install-prereqs-ubuntu.sh` created a different one (commonly `~/venv-ardupilot` already, or check with `which python3`), either:

- rename/symlink it to match, or
- set **Venv activate** in the GUI's Setup tab to point at wherever it actually is.

Activate it and confirm ArduPilot's Python deps are present:

```sh
source ~/venv-ardupilot/bin/activate
pip install -r requirements.txt   # from the ardupilot repo root, if not already done
```

## 5. Build SITL once

First build must NOT use `--no-rebuild` (the GUI always passes `--no-rebuild`, assuming a binary already exists). `~/ardupilot` below is the default — swap it for your own clone path if you set a custom one in the GUI's Setup tab:

```sh
cd ~/ardupilot
source ~/venv-ardupilot/bin/activate
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad
```

Let it finish building, then `Ctrl+C` once you see it running (MAVProxy prompt `>` appears) — this run was just to compile the binary. Rebuild the same way any time you change branch/version; day-to-day launches (via this repo's GUI or manually) can then use `--no-rebuild` for a fast start.

## 6. Enable Lua scripting (optional)

Only needed if using the [fault-injection scripts](../../Lua-scripts-fault-journal-paper/lua-script-readme.md):

```sh
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild -w
# once it boots, in the MAVProxy prompt:
param set SCR_ENABLE 1
param fetch
```

Then copy your `.lua` script into `<ardupilot_dir>/scripts/` before the next launch.

## 7. Run it

**Via this repo's GUI** (Windows + WSL only): Setup tab → pick param file → Launch. This runs both the DT backend (`Src/DT/main.py`) and, inside WSL, the exact command below — nothing to type by hand. The Launch button shells out to `wsl.exe`/Windows Terminal directly, so it only works on Windows.

Order of events on Launch: the launcher saves your settings, checks the ArduPilot dir in WSL, starts the DT (`Src/DT/main.py`, telemetry on) and only then opens the WSL terminal — so the DT is already listening on UDP 9002 when ArduPilot's first frame is sent. **Host IP:** the field starts at `127.0.0.1`; on Windows + WSL press **Detect IP** first (the server only auto-detects when the field is empty). **Wipe EEPROM** ticks `-w`; leave it off and parameters from earlier runs persist in ArduPilot's `eeprom.bin` and can shadow the `.parm`. MAVLink is also forwarded to `udp:<host ip>:14551` for any ground-control software.

First launch copies the selected `.parm` file into WSL at `~/.dt_params/<param_file>` (CRLF stripped) and ArduPilot reads it from there — you'll see `SIM_VEHICLE: Adding parameters from (/home/<user>/.dt_params/<param_file>)` in the launch terminal. That's expected, not an error; it re-copies (overwriting the cached one) on every launch, so editing the source `.parm` file in this repo always takes effect on the next Launch.

**Manually / headless** (required on native Linux/macOS — the GUI Launch button doesn't apply there), run the DT first (it opens the UDP socket ArduPilot connects to):

```sh
# from the KTP-DT repo root
.venv/bin/python Src/DT/main.py
```

Then in a separate WSL/Linux shell, launch ArduPilot's JSON backend pointed at the DT's host IP and UDP port (`9002` by default, from `Src/DT/config/startup_config.json`). `~/ardupilot` below is the default clone path — use your own if different:

```sh
cd ~/ardupilot
source ~/venv-ardupilot/bin/activate
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild \
  --model JSON:<DT_HOST_IP> --out=udp:<DT_HOST_IP>:14551 \
  --add-param-file=<path-to-parm-file>
```

`<DT_HOST_IP>` — on native Linux/macOS this is `127.0.0.1`. On Windows/WSL, use the IP printed by `ip route show default | awk '{print $3}'` inside WSL (the GUI's "Detect IP" button does this for you).

## Troubleshooting

- **"ArduPilot not found at \<path\> — check the path in Setup"**: the GUI checks the configured ArduPilot dir exists (and looks like a real ArduPilot clone) before launching. Fix the **ArduPilot dir** field in the Setup tab.
- **"UDP port 9002 already in use"**: a previous DT/SITL pair is still running. Stop it (GUI Stop button, or `pkill -f main.py` / `pkill -f sim_vehicle.py`). The launcher's check binds a TCP socket, so a stale *UDP* listener may slip through — if the new DT then stays silent, look for a leftover `python main.py`.
- **WSL can't reach the DT / no telemetry**: Windows Firewall can block inbound UDP from WSL's virtual NIC. Allow Python through the firewall, or temporarily disable the firewall to confirm that's the cause.
- **`git submodule update` fails / missing files under `modules/`**: re-run `git submodule update --init --recursive` from the ardupilot repo root; a shallow or interrupted clone is the usual cause.
- **Build fails on missing packages**: re-run `Tools/environment_install/install-prereqs-ubuntu.sh -y`, then open a fresh shell so `~/.profile` PATH changes take effect.
- **MAVProxy immediately exits / "unable to connect"**: the DT (`main.py`) must already be listening before ArduPilot's JSON backend connects — start the DT first, or use the GUI which sequences this automatically.
