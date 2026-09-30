"""Launch and stop local backend plus optional WSL flight controller."""
from __future__ import annotations
import json, os, re, shlex, shutil, socket, subprocess, sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
STATE_FILE = REPO_ROOT / "Results" / "gui_state.json"
DEFAULT_ARDUPILOT_DIR = "~/ardupilot"
DEFAULT_VENV_ACTIVATE = "~/venv-ardupilot/bin/activate"

def load_state():
    try: return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except Exception: return {}

def save_state(state):
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True); STATE_FILE.write_text(json.dumps(state, indent=2), encoding="utf-8")

def get_host_ip(force=False):
    state = load_state()
    if state.get("host_ip") and not force: return state["host_ip"]
    result = subprocess.run(["wsl.exe", "-e", "bash", "-lc", "ip route show default | awk '{print $3}'"], capture_output=True, text=True, timeout=15)
    ip = next((line.strip() for line in result.stdout.splitlines() if line.strip()), "")
    if not ip: raise RuntimeError("could not detect WSL host IP")
    state["host_ip"] = ip; save_state(state); return ip

def _port_free(port):
    with socket.socket() as sock:
        try: sock.bind(("127.0.0.1", port)); return True
        except OSError: return False

def _wsl_path(path):
    unc = re.match(r"^\\\\(?:wsl\.localhost|wsl\$)\\[^\\]+\\(.*)$", path.strip(), re.I)  # Explorer-copied UNC -> Linux path
    if unc: path = "/" + unc.group(1).replace("\\", "/")
    if path.startswith("~/"): return '"$HOME/' + path[2:].replace('"', '\\"') + '"'
    return shlex.quote(path)

def _validate_wsl_dir(path):
    check = f"test -d {_wsl_path(path)} && test -f {_wsl_path(path)}/Tools/autotest/sim_vehicle.py"
    result = subprocess.run(["wsl.exe", "-e", "bash", "-lc", check], capture_output=True, timeout=15)
    if result.returncode != 0: raise RuntimeError(f"ArduPilot not found at {path} — check the path in Setup")

def _param_dir_wsl():
    drive = REPO_ROOT.drive[:1].lower(); relative = str(REPO_ROOT)[3:].replace("\\", "/")
    return f"/mnt/{drive}/{relative}/Src/DT/config/drone_sim_param_files"

def _build_sitl_cmd(param_file, ip, ardupilot_dir, venv_activate, preserve_eeprom=False):
    src = shlex.quote(f"{_param_dir_wsl()}/{param_file}"); dst = f'"$HOME/.dt_params/{param_file}"'; wipe = "" if preserve_eeprom else " -w"
    return f"cd {_wsl_path(ardupilot_dir)} && source {_wsl_path(venv_activate)} && mkdir -p \"$HOME/.dt_params\" && tr -d '\\r' < {src} > {dst} && Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild --model JSON:{ip} --out=udp:{ip}:14551 --add-param-file={dst}{wipe}"

class LaunchManager:
    def __init__(self): self._backend = None; self._sitl_running = False; self._last_ip = None; self._last_param = None
    def status(self):
        state = load_state()
        return {"backend_running": self._backend is not None and self._backend.poll() is None, "sitl_running": self._sitl_running, "host_ip": self._last_ip or state.get("host_ip"), "param_file": self._last_param, "ardupilot_dir": state.get("ardupilot_dir", DEFAULT_ARDUPILOT_DIR), "venv_activate": state.get("venv_activate", DEFAULT_VENV_ACTIVATE)}
    def launch(self, param_file, ip=None, venv_activate=None, ardupilot_dir=None, config=None):
        if self._backend is not None and self._backend.poll() is None: raise RuntimeError("backend already running")
        ip = ip or get_host_ip(); state = load_state(); ardupilot_dir = ardupilot_dir or state.get("ardupilot_dir", DEFAULT_ARDUPILOT_DIR); venv_activate = venv_activate or state.get("venv_activate", DEFAULT_VENV_ACTIVATE)
        state.update(host_ip=ip, ardupilot_dir=ardupilot_dir, venv_activate=venv_activate); save_state(state)
        if not _port_free(9002): raise RuntimeError("UDP port 9002 already in use")
        _validate_wsl_dir(ardupilot_dir)
        self._backend = subprocess.Popen([sys.executable, "main.py"], cwd=REPO_ROOT / "Src" / "DT", env={**os.environ, "DT_TELEMETRY_ENABLED": "1"})
        cfg = config or {}; preserve = bool(cfg.get("sitl", {}).get("preserve_eeprom", False))
        cmd = _build_sitl_cmd(param_file, ip, ardupilot_dir, venv_activate, preserve)
        args = ["wsl.exe", "-e", "bash", "-lc", cmd]; wt = shutil.which("wt.exe")
        if wt: subprocess.Popen([wt, "-w", "0", "nt", *args])
        else: subprocess.Popen(args, creationflags=subprocess.CREATE_NEW_CONSOLE)  # cmd /c start mangled the quoted title + bash -lc quoting
        self._last_ip, self._last_param, self._sitl_running = ip, param_file, True; return self.status()
    def stop(self):
        if self._backend is not None:
            if self._backend.poll() is None:
                if sys.platform == "win32": subprocess.run(["taskkill", "/F", "/T", "/PID", str(self._backend.pid)], capture_output=True)
                else: self._backend.terminate()
            self._backend = None
        if self._sitl_running:
            try: subprocess.run(["wsl.exe", "-e", "bash", "-lc", "pkill -INT -f sim_vehicle.py; sleep 1; pkill -9 -f 'sim_vehicle.py|arducopter|mavproxy'"], timeout=20, capture_output=True)
            except Exception: pass
            self._sitl_running = False
        return self.status()
