"""Config persistence and dropdown values."""
from __future__ import annotations
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "Src" / "DT" / "config"
DRONE_CFG_DIR = CONFIG_DIR / "drone_config_variables"
PARAM_DIR = CONFIG_DIR / "drone_sim_param_files"
DESIGN_DIR = REPO_ROOT / "Src" / "Design-config-tool"
if str(DESIGN_DIR) not in sys.path: sys.path.insert(0, str(DESIGN_DIR))

def config_path(name):
    if name == "startup": return CONFIG_DIR / "startup_config.json"
    if name.startswith("drone:"): return DRONE_CFG_DIR / f"{name.split(':', 1)[1]}.json"
    raise KeyError(f"unknown config name: {name!r}")

def read_config(name): return json.loads(config_path(name).read_text(encoding="utf-8"))

def write_config(name, data):
    if isinstance(data, str):
        try: data = json.loads(data)
        except json.JSONDecodeError as exc: raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, (dict, list)): raise ValueError("config must be JSON object or array")
    path = config_path(name); path.parent.mkdir(parents=True, exist_ok=True); tmp = path.with_suffix(".tmp"); tmp.write_text(json.dumps(data, indent=2), encoding="utf-8"); tmp.replace(path); return path

def options():
    from db_schema import get_battery_names, get_motor_names
    return {"param_files": sorted(p.name for p in PARAM_DIR.iterdir() if p.suffix in {".parm", ".param"}) if PARAM_DIR.is_dir() else [], "drone_models": sorted(p.stem for p in DRONE_CFG_DIR.glob("*.json")), "motors": get_motor_names(), "batteries": get_battery_names()}
