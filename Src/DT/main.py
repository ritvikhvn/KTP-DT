"""ArduPilot JSON connector entry point."""
from __future__ import annotations
import json
import signal
import sys
import time
from multiprocessing import Process, Queue
from pathlib import Path

from Simulation.simulation import Simulation
from Simulation.flight_controller.ardupilot_sitl_connector import SITLConnector
from Simulation.visualisation.logger import StateLogger
from Simulation.visualisation.viz3d import run_viz
from Simulation.visualisation.telemetry_pub import make_telemetry_publisher

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = REPO_ROOT / "Src" / "DT" / "config"

def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))

def _build_telemetry(t, state, sim):
    return {"t": t, "accel": state.accel_bb.tolist(), "gyro": state.omega_body.tolist(), "gps_alt": state.lla[2], "gps_vn": float(state.v_ned[0]), "baro_alt": state.lla[2], "state_alt": state.lla[2], "wind_ned": sim.v_gust_ned.tolist(), "mag_xyz": None}

def main() -> int:
    startup = load_json(CONFIG_ROOT / "startup_config.json")
    drone = load_json(CONFIG_ROOT / "drone_config_variables" / f"{startup['drone_model'].replace('-', '_')}.json")
    sim = Simulation(startup, drone)
    sitl = startup["sitl"]
    link = SITLConnector(int(sitl["udp_port"]), int(sitl.get("print_every", 1000)), bool(sitl.get("verbose", False)), float(startup["sim"]["sim_rate"]))
    logger = None
    if startup.get("logging", {}).get("enabled"):
        logger = StateLogger(REPO_ROOT / startup["logging"]["path"], rotor_channels=[r.pwm_ch for r in sim.vehicle.rotors])
    viz = startup.get("viz", {}); viz_q = Queue(maxsize=1) if viz.get("enabled") else None; viz_proc = None
    if viz_q is not None:
        viz_proc = Process(target=run_viz, args=(viz_q, float(viz.get("span_m", 20))), daemon=True); viz_proc.start()
    tele = make_telemetry_publisher(startup)
    sim_time = 0.0; last_realtime = 0.0; viz_last = 0.0; viz_dt = 1.0 / float(viz.get("fps", 30))
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        while True:
            rx = link.recv_pwm()
            if rx.reset: sim.reset(); sim_time = 0.0
            dt = min(1.0 / max(rx.frame_rate, 1), 1.0 / float(startup["sim"]["sim_rate"]))
            started = time.perf_counter(); state = sim.tick(rx.pwm, dt); tick_us = (time.perf_counter() - started) * 1e6; sim_time += dt
            link.send_state(state, sim_time)
            payload = _build_telemetry(sim_time, state, sim)
            if tele is not None: tele.maybe_send(sim_time, {"type": "telemetry", **payload, "pos_ned": state.x_ned.tolist(), "quat_wxyz": state.quat.tolist()})
            stats = link.report_progress(sim_time, pwm=rx.pwm, state=state, tick_us=tick_us)
            if stats and tele is not None: last_realtime = stats["realtime_pct"]; tele.send({"type": "status", **stats})
            if viz_q is not None and sim_time - viz_last >= viz_dt:
                viz_last = sim_time
                try: viz_q.put_nowait({"t": sim_time, "pos_ned": state.x_ned.tolist(), "quat_wxyz": state.quat.tolist()})
                except Exception: pass
            if logger is not None: logger.write(sim_time, state, pwm=rx.pwm, realtime_pct=last_realtime)
    finally:
        if logger is not None: logger.close()
        if viz_proc is not None: viz_proc.terminate(); viz_proc.join(timeout=1)
        if tele is not None: tele.close()
        link.close()
    return 0

if __name__ == "__main__": sys.exit(main())
