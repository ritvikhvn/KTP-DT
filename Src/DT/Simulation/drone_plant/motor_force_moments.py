"""Rotor thrust, reaction moment, and actuator lag."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ..constants import G

@dataclass
class RotorSpec:
    pwm_ch: int
    arm_pos: np.ndarray
    spin_dir: int
    motor: object

def pwm_to_throttle(pwm, rotors, pwm_min=1000, pwm_max=2000):
    if not rotors: return np.zeros(0)
    if len(pwm) < max(r.pwm_ch for r in rotors): raise ValueError("PWM array too short")
    scale = float(pwm_max - pwm_min)
    return np.asarray([np.clip((float(pwm[r.pwm_ch - 1]) - pwm_min) / scale, 0.0, 1.0) for r in rotors])

def compute_motor_forces(throttle, rotors, g_const=G, f_body_out=None, m_body_out=None, per_thrust_out=None, per_power_out=None, per_current_out=None, per_rpm_out=None):
    n = len(rotors)
    force = np.zeros(3) if f_body_out is None else f_body_out; moment = np.zeros(3) if m_body_out is None else m_body_out
    thrusts = np.zeros(n) if per_thrust_out is None else per_thrust_out; powers = np.zeros(n) if per_power_out is None else per_power_out
    currents = np.zeros(n) if per_current_out is None else per_current_out; rpms = np.zeros(n) if per_rpm_out is None else per_rpm_out
    force.fill(0); moment.fill(0)
    for i, rotor in enumerate(rotors):
        t = float(throttle[i]); thrust = rotor.motor.interp_thrust_g(t) * g_const / 1000.0
        force[2] -= thrust; moment += np.cross(rotor.arm_pos, [0, 0, -thrust]); moment[2] += rotor.motor.interp_torque_nm(t) * rotor.spin_dir
        thrusts[i] = thrust; powers[i] = rotor.motor.interp_power_w(t); currents[i] = rotor.motor.interp_current_a(t); rpms[i] = rotor.motor.interp_rpm(t)
    return force, moment, thrusts, powers, currents, rpms

class MotorDynamics:
    def __init__(self, n_rotors, cfg, dt):
        self.n_rotors = int(n_rotors); self.dt = dt; self.enabled = bool(cfg.get("enabled", False)); self.tau_s = float(cfg.get("motor_tau_s", 0.07)); self.throttle_state = np.zeros(self.n_rotors)
        self._startup_snapshot = {"enabled": self.enabled, "tau_s": self.tau_s}
    def set_param(self, key, value): setattr(self, key, bool(value) if key == "enabled" else float(value))
    def reset_to_startup(self):
        self.enabled, self.tau_s = self._startup_snapshot["enabled"], self._startup_snapshot["tau_s"]
    def reset(self): self.throttle_state.fill(0)
    def step(self, commanded, dt):
        if not self.enabled or self.tau_s <= 0: self.throttle_state[:] = commanded; return commanded
        alpha = 1.0 - np.exp(-float(dt) / max(self.tau_s, 1e-6)); self.throttle_state += alpha * (commanded - self.throttle_state); return self.throttle_state
