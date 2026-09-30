"""Tests for MotorDynamics first-order actuator lag + pwm_to_throttle.

Run from repo root: pytest tests/test_motor_dynamics.py
"""
import numpy as np
import pytest

from Simulation.drone_plant.motor_force_moments import MotorDynamics, pwm_to_throttle, RotorSpec


class _FakeMotor:
    """Minimal MotorLookup stand-in: linear thrust, used only by pwm_to_throttle tests."""
    def interp_thrust_g(self, t):
        return 1000.0 * t
    def interp_torque_nm(self, t):
        return 0.1 * t
    def interp_power_w(self, t):
        return 100.0 * t
    def interp_current_a(self, t):
        return 0.0


def _rotors(n=4):
    return [
        RotorSpec(pwm_ch=i + 1, arm_pos=np.zeros(3), spin_dir=(1 if i % 2 else -1), motor=_FakeMotor())
        for i in range(n)
    ]


# ---------------------------------------------------------------------------
# pwm_to_throttle
# ---------------------------------------------------------------------------
def test_pwm_to_throttle_normalises_and_clips():
    rotors = _rotors(4)
    pwm = np.array([1000, 1500, 2000, 2500] + [1500] * 12, dtype=float)  # ch4 above max → clip 1.0
    th = pwm_to_throttle(pwm, rotors, 1000, 2000)
    assert np.allclose(th, [0.0, 0.5, 1.0, 1.0])


def test_pwm_to_throttle_picks_correct_channel():
    rotors = _rotors(2)
    rotors[0].pwm_ch = 3
    rotors[1].pwm_ch = 7
    pwm = np.zeros(16) + 1000.0
    pwm[2] = 2000.0  # ch3
    pwm[6] = 1500.0  # ch7
    th = pwm_to_throttle(pwm, rotors, 1000, 2000)
    assert np.allclose(th, [1.0, 0.5])


def test_pwm_to_throttle_raises_when_array_too_short():
    rotors = _rotors(2)
    rotors[1].pwm_ch = 9
    with pytest.raises(ValueError):
        pwm_to_throttle(np.zeros(4), rotors, 1000, 2000)


# ---------------------------------------------------------------------------
# MotorDynamics — disabled passthrough
# ---------------------------------------------------------------------------
def test_disabled_is_passthrough():
    md = MotorDynamics(4, {"enabled": False, "motor_tau_s": 0.03}, dt=0.0025)
    cmd = np.array([0.1, 0.5, 0.9, 0.3])
    out = md.step(cmd, 0.0025)
    assert out is cmd  # exact passthrough, no filtering


def test_tau_zero_is_passthrough():
    md = MotorDynamics(2, {"enabled": True, "motor_tau_s": 0.0}, dt=0.0025)
    cmd = np.array([0.2, 0.8])
    assert md.step(cmd, 0.0025) is cmd


# ---------------------------------------------------------------------------
# MotorDynamics — first-order step response
# ---------------------------------------------------------------------------
def _step_response(md, target, dt, n_steps):
    cmd = np.full(md.n_rotors, target)
    out = None
    for _ in range(n_steps):
        out = md.step(cmd, dt)
    return out


def test_step_reaches_63pct_at_tau():
    tau, dt, target = 0.1, 0.001, 1.0
    md = MotorDynamics(1, {"enabled": True, "motor_tau_s": tau}, dt=dt)
    out = _step_response(md, target, dt, n_steps=int(round(tau / dt)))  # t = tau
    assert out[0] == pytest.approx(1.0 - np.exp(-1.0), abs=1e-3)  # ~0.632


def test_step_reaches_95pct_at_3tau():
    tau, dt, target = 0.1, 0.001, 1.0
    md = MotorDynamics(1, {"enabled": True, "motor_tau_s": tau}, dt=dt)
    out = _step_response(md, target, dt, n_steps=int(round(3 * tau / dt)))  # t = 3*tau
    assert out[0] == pytest.approx(1.0 - np.exp(-3.0), abs=1e-3)  # ~0.950


def test_lag_output_trails_command():
    md = MotorDynamics(1, {"enabled": True, "motor_tau_s": 0.05}, dt=0.0025)
    out = md.step(np.array([1.0]), 0.0025)
    assert 0.0 < out[0] < 1.0  # one step: partway, not instantaneous


def test_variable_dt_coefficient():
    """Two small steps of dt and one big step of 2*dt converge differently but
    the per-call coefficient must use the dt passed, not a fixed one."""
    tau = 0.04
    md_a = MotorDynamics(1, {"enabled": True, "motor_tau_s": tau}, dt=0.01)
    md_a.step(np.array([1.0]), 0.01)
    out_a = md_a.step(np.array([1.0]), 0.01)
    md_b = MotorDynamics(1, {"enabled": True, "motor_tau_s": tau}, dt=0.01)
    out_b = md_b.step(np.array([1.0]), 0.02)  # single double-length step
    # exact: after two 0.01 steps a=exp(-0.01/tau); after one 0.02 step a=exp(-0.02/tau)
    a = np.exp(-0.01 / tau)
    expect_a = (1 - a) + a * (1 - a)  # state after two steps from 0, cmd=1
    assert out_a[0] == pytest.approx(expect_a, abs=1e-9)
    assert out_b[0] == pytest.approx(1 - np.exp(-0.02 / tau), abs=1e-9)


# ---------------------------------------------------------------------------
# Runtime control surface
# ---------------------------------------------------------------------------
def test_set_param_and_reset_to_startup():
    md = MotorDynamics(2, {"enabled": False, "motor_tau_s": 0.03}, dt=0.0025)
    md.set_param("enabled", True)
    md.set_param("tau_s", 0.05)
    assert md.enabled is True and md.tau_s == 0.05
    md.reset_to_startup()
    assert md.enabled is False and md.tau_s == 0.03


def test_set_param_coerces_string_values():
    md = MotorDynamics(1, {"enabled": False, "motor_tau_s": 0.03}, dt=0.0025)
    md.set_param("tau_s", "0.07")     # JSON/ZMQ string path
    md.set_param("enabled", "true")
    assert md.tau_s == 0.07 and md.enabled is True


def test_reset_zeroes_state():
    md = MotorDynamics(3, {"enabled": True, "motor_tau_s": 0.03}, dt=0.0025)
    md.step(np.array([1.0, 1.0, 1.0]), 0.0025)
    assert np.any(md.throttle_state > 0.0)
    md.reset()
    assert np.allclose(md.throttle_state, 0.0)
