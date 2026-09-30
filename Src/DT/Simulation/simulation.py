"""Six-degree-of-freedom vehicle simulation."""
from __future__ import annotations

from dataclasses import dataclass
import sys
from pathlib import Path
import numpy as np

_SRC_DIR = Path(__file__).resolve().parents[2]
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from .state import StateVector
from .drone_plant import sixdof_quaternion
from .drone_plant.sixdof_quaternion import Integrator
from .drone_plant.motor_force_moments import RotorSpec, compute_motor_forces, MotorDynamics, pwm_to_throttle
from .drone_plant import motor_force_moments as _motor_module
from .drone_plant import environment_aero as _aero_module
from .drone_plant import ground_contact as _ground_module
from .drone_plant.environment_aero import aero_drag
from .drone_plant.ground_contact import ground_forces
from .drone_plant.dryden_turbulence import DrydenTurbulence
from .constants import G, G_NED as _G_NED
from database.design_config_tool_import.motor_battery_db import load_databases, MotorLookup, MotorRecord


@dataclass
class VehicleParams:
    mass: float
    inertia: np.ndarray
    inertia_inv: np.ndarray
    cg: np.ndarray
    cg_ground_offset: float
    drag_cd: np.ndarray
    drag_cd_moment: np.ndarray
    surface_area_xyz: np.ndarray
    rotors: list[RotorSpec]
    pwm_min: int
    pwm_max: int
    ground_k: float
    ground_c: float
    ground_mu: float
    ground_lat_damp: float
    ground_rot_damp: float


class Simulation:
    def __init__(self, startup_cfg: dict, drone_cfg: dict):
        self.startup_cfg = startup_cfg
        self.drone_cfg = drone_cfg
        self.rho = float(startup_cfg["environment"]["rho"])
        self.g = G
        self.lla0 = np.asarray(startup_cfg["initial"]["pos_lla"], dtype=float)
        self._Ts = 1.0 / float(startup_cfg["sim"]["sim_rate"])
        self._sensor_seeds = self._init_seeds(startup_cfg)
        self._init_dryden(startup_cfg)
        self.vehicle = _build_vehicle(drone_cfg)
        self._motor_dyn = MotorDynamics(len(self.vehicle.rotors), startup_cfg.get("actuator", {}), self._Ts)
        self._initial_cfg = startup_cfg["initial"]
        self.state = StateVector.from_initial(self._initial_cfg)
        self._integrator = Integrator()
        self.v_gust_ned = np.zeros(3)
        self._alloc_tick_buffers()

    def _init_seeds(self, startup_cfg: dict) -> dict:
        master = startup_cfg.get("sim", {}).get("seed")
        if master is None:
            return {key: None for key in ("gps", "imu", "baro", "mag", "dryden")}
        children = np.random.SeedSequence(int(master)).spawn(5)
        return dict(zip(("gps", "imu", "baro", "mag", "dryden"), children))

    def _init_dryden(self, startup_cfg: dict) -> None:
        cfg = startup_cfg["environment"].get("dryden", {})
        self._dryden_enabled = bool(cfg.get("enabled", False))
        seed_seq = self._sensor_seeds["dryden"]
        self._dryden = DrydenTurbulence(
            W20=float(cfg.get("W20_ms", 15.0)),
            wind_heading_deg=float(cfg.get("wind_heading_deg", 0.0)),
            horizontal_wind_speed_ms=float(cfg.get("horizontal_wind_speed_ms", 0.0)),
            vertical_wind_speed_ms=float(cfg.get("vertical_wind_speed_ms", 0.0)),
            dt_filt=float(cfg.get("dt_filt", self._Ts)),
            seeds=None if seed_seq is not None else cfg.get("seeds"),
            seed_seq=seed_seq,
        )

    def _alloc_tick_buffers(self) -> None:
        n = len(self.vehicle.rotors)
        self._throttle_buf = np.zeros(n)
        self._f_motors_buf = np.zeros(3)
        self._m_motors_buf = np.zeros(3)
        self._f_total = np.zeros(3)
        self._m_total = np.zeros(3)
        self._v_gust_zeros = np.zeros(3)

    def reset(self) -> None:
        self._dryden.reset()
        self._dryden.reset_wind_to_startup()
        self._motor_dyn.reset()
        self._motor_dyn.reset_to_startup()
        self.state = StateVector.from_initial(self._initial_cfg)

    def tick(self, pwm: np.ndarray, dt: float) -> StateVector:
        s, v = self.state, self.vehicle
        throttle = pwm_to_throttle(pwm, v.rotors, v.pwm_min, v.pwm_max)
        throttle = self._motor_dyn.step(throttle, dt)
        compute_motor_forces(throttle, v.rotors, self.g, self._f_motors_buf, self._m_motors_buf)
        _, _, on_ground = ground_forces(s.x_ned, s.v_ned, s.dcm_be, s.omega_body, v.cg_ground_offset, v.ground_k, v.ground_c, v.ground_mu, v.ground_lat_damp, v.ground_rot_damp)
        gust = self._v_gust_zeros
        if not on_ground and self._dryden_enabled:
            gust = self._dryden.update(s.x_ned, s.dcm_be, float(np.linalg.norm(s.v_body)), self._Ts)
        self.v_gust_ned = gust
        aero_drag(s.v_ned, s.omega_body, s.dcm_be, gust, self.rho, v.drag_cd, v.surface_area_xyz, v.drag_cd_moment, v.mass, _G_NED)
        f_ground, m_ground = _ground_module._f_gnd_s, _ground_module._m_gnd_s
        f_aero, m_aero, f_gravity = _aero_module._f_drag_s, _aero_module._m_drag_s, _aero_module._f_gravity_s
        self._f_total[:] = self._f_motors_buf + f_aero + f_ground + f_gravity
        self._m_total[:] = self._m_motors_buf + m_aero + m_ground
        sixdof_quaternion.step(s, self._f_total, self._m_total, v.mass, v.inertia, v.inertia_inv, dt, scratch=self._integrator, lla0=self.lla0)
        return s


def _build_vehicle(drone_cfg: dict) -> VehicleParams:
    af = drone_cfg["airframe"]
    ic = af["inertia_cad_kgmm2"]
    inertia = np.array([[ic["Ixx"], ic["Ixy"], ic["Ixz"]], [ic["Ixy"], ic["Iyy"], ic["Iyz"]], [ic["Ixz"], ic["Iyz"], ic["Izz"]]], dtype=float) / 1e6
    motors, _ = load_databases()
    index = _build_motor_index(motors)
    cg = np.asarray(af["cg"], dtype=float) / 1000.0
    rotors = []
    for item in drone_cfg["rotors"]["control"]:
        record = index.get((item["motor_name"].lower(), "control"))
        if record is None:
            raise ValueError(f"Motor '{item['motor_name']}' not found")
        rotors.append(RotorSpec(int(item["pwm_ch"]), np.asarray(item["arm_pos_centroid"], dtype=float) / 1000.0 - cg, int(item["spin_dir"]), MotorLookup.from_record(record)))
    gc = drone_cfg.get("ground_contact", {})
    pwm = drone_cfg.get("pwm", {"min": 1000, "max": 2000})
    return VehicleParams(float(af["mass"]), inertia, np.linalg.inv(inertia), cg, float(af["cg_ground_offset"]) / 1000.0, np.asarray(af["drag_cd"], dtype=float), np.asarray(af["drag_cd_moment"], dtype=float), np.asarray(af["surface_area_xyz"], dtype=float), rotors, int(pwm["min"]), int(pwm["max"]), float(gc.get("k_spring", 500)), float(gc.get("c_damp", 50)), float(gc.get("mu_friction", 0.8)), float(gc.get("k_lat_damp", 40)), float(gc.get("k_rot_damp", 5)))


def _build_motor_index(motors: list[MotorRecord]) -> dict[tuple[str, str], MotorRecord]:
    index = {}
    for motor in motors:
        index.setdefault((motor.name.lower(), motor.type.lower()), motor)
    return index
