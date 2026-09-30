"""FastAPI application for setup, telemetry, and flight-time views."""
from __future__ import annotations
import asyncio, sys
from pathlib import Path
import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from . import config_io
from .launcher import LaunchManager, get_host_ip
from .telemetry import TelemetryHub

STATIC_DIR = Path(__file__).resolve().parent / "static"
PROPULSION_DIR = str(Path(__file__).resolve().parent.parent / "Src" / "Propulsion-model")

def _jsonable(obj):
    if isinstance(obj, dict): return {k: _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)): return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray): return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)): return obj.item()
    return obj

def create_app():
    app = FastAPI(title="KTP Digital Twin System")
    launcher = LaunchManager(); telemetry = TelemetryHub()
    @app.on_event("startup")
    async def startup(): telemetry.start()
    @app.on_event("shutdown")
    async def shutdown(): await telemetry.stop(); launcher.stop()
    @app.get("/api/options")
    async def options(): return config_io.options()
    @app.get("/api/config/{name}")
    async def read_config(name):
        try: return await asyncio.to_thread(config_io.read_config, name)
        except (KeyError, FileNotFoundError) as exc: raise HTTPException(404, str(exc)) from exc
    @app.put("/api/config/{name}")
    async def write_config(name, body: dict):
        try: path = await asyncio.to_thread(config_io.write_config, name, body.get("data", body))
        except (KeyError, ValueError) as exc: raise HTTPException(400, str(exc)) from exc
        return {"ok": True, "path": str(path)}
    @app.get("/api/status")
    async def status(): return launcher.status()
    @app.get("/api/detect-ip")
    async def detect_ip(force: bool = False):
        try: return {"host_ip": await asyncio.to_thread(get_host_ip, force)}
        except Exception as exc: raise HTTPException(500, str(exc)) from exc
    @app.post("/api/launch")
    async def launch(body: dict):
        if not body.get("param_file"): raise HTTPException(400, "param_file required")
        if body.get("config"): await asyncio.to_thread(config_io.write_config, "startup", body["config"])
        try: return await asyncio.to_thread(launcher.launch, body["param_file"], ip=body.get("ip"), venv_activate=body.get("venv_activate"), ardupilot_dir=body.get("ardupilot_dir"), config=body.get("config"))
        except Exception as exc: raise HTTPException(409, str(exc)) from exc
    @app.post("/api/stop")
    async def stop(): return await asyncio.to_thread(launcher.stop)
    @app.post("/api/flight-time")
    async def flight_time(body: dict):
        design_dir = str(Path(__file__).resolve().parent.parent / "Src" / "Design-config-tool")
        if design_dir not in sys.path: sys.path.insert(0, design_dir)
        from flight_time.flight_time_calc import calculate_flight_time
        try: return await asyncio.to_thread(calculate_flight_time, body, False)
        except (KeyError, ValueError, IndexError) as exc: raise HTTPException(400, str(exc)) from exc
    @app.post("/api/propulsion/motor")
    async def propulsion_motor(body: dict):
        if PROPULSION_DIR not in sys.path: sys.path.insert(0, PROPULSION_DIR)
        from aerodynamic_model.motor import solve_motor_equilibrium
        try:
            kv, rm, i0 = float(body["kv"]), float(body["rm"]), float(body["i0"])
            v_batt, throttle = float(body["v_batt"]), float(body["throttle"])
            eta_esc, k_iron = float(body.get("eta_esc", 1.0)), float(body.get("k_iron", 0.0))
            t_winding = float(body.get("t_winding_c", 20.0))
            load_torque = float(body.get("load_torque_nm", 0.0))
            point = await asyncio.to_thread(solve_motor_equilibrium, kv, rm, i0, v_batt, throttle, load_torque_Nm=load_torque, eta_ESC=eta_esc, k_iron=k_iron, T_winding_C=t_winding)
            thr_sweep = np.linspace(0.1, 1.0, 19)
            sweep = [await asyncio.to_thread(solve_motor_equilibrium, kv, rm, i0, v_batt, float(t), load_torque_Nm=load_torque, eta_ESC=eta_esc, k_iron=k_iron, T_winding_C=t_winding) for t in thr_sweep]
            point["sweep"] = {"throttle": thr_sweep, "rpm": [s["rpm"] for s in sweep], "current_A": [s["current_A"] for s in sweep], "torque_Nm": [s["torque_Nm"] for s in sweep], "eta_motor": [s["eta_motor"] for s in sweep]}
            return _jsonable(point)
        except (KeyError, ValueError) as exc: raise HTTPException(400, str(exc)) from exc
    @app.post("/api/propulsion/propeller")
    async def propulsion_propeller(body: dict):
        if PROPULSION_DIR not in sys.path: sys.path.insert(0, PROPULSION_DIR)
        from aerodynamic_model.geometry import get_blade_geometry
        from aerodynamic_model.airfoil import get_airfoil_polars
        from aerodynamic_model.single_rotor import solve_single_rotor
        try:
            d_m, p_m = float(body["diameter_in"]) * 0.0254, float(body["pitch_in"]) * 0.0254
            blades, airfoil, rpm = int(body["blades"]), str(body.get("airfoil", "clark_y")), float(body["rpm"])
            v_climb = float(body.get("v_climb", 0.0))
            geo = await asyncio.to_thread(get_blade_geometry, d_m, p_m, blades, rpm, 5.0)
            polars = await asyncio.to_thread(get_airfoil_polars, airfoil)
            result = await asyncio.to_thread(solve_single_rotor, geo, polars, rpm, V_climb=v_climb, return_radial=True)
            result["r_norm"] = result["r"] / geo["R"]
            result["source"], result["apc_file"] = geo["source"], geo["apc_file"]
            return _jsonable(result)
        except (KeyError, ValueError, NotImplementedError) as exc: raise HTTPException(400, str(exc)) from exc
        except Exception as exc: raise HTTPException(400, f"airfoil {body.get('airfoil')!r} could not be resolved: {exc}") from exc
    @app.websocket("/ws/telemetry")
    async def websocket(ws: WebSocket):
        await ws.accept(); telemetry.add_client(ws)
        try:
            while True: await ws.receive_text()
        except (WebSocketDisconnect, Exception): pass
        finally: telemetry.remove_client(ws)
    @app.middleware("http")
    async def no_cache(request, call_next):
        # Dev GUI: a stale cached app.js paired with a fresh index.html silently breaks
        # the page (old JS querying element IDs the new markup doesn't have). Staleness
        # costs more here than the caching saves.
        response = await call_next(request)
        response.headers["Cache-Control"] = "no-store"
        return response
    app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
    return app
