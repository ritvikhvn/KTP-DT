"""Quaternion 6-DOF rigid-body integrator."""
from __future__ import annotations
import numpy as np
from ..state import StateVector, _quat_to_dcm_eb

class Integrator:
    pass

def _rhs(y, force, moment, mass, inertia, inertia_inv):
    q = y[6:10]; q = q / max(float(np.linalg.norm(q)), 1e-12); dcm = _quat_to_dcm_eb(q)
    vel, omega = y[3:6], y[10:13]
    out = np.zeros(13); out[:3] = dcm @ vel; out[3:6] = force / mass - np.cross(omega, vel)
    w, x, yq, z = q; p, qrate, r = omega
    out[6:10] = 0.5 * np.array([-p*x-qrate*yq-r*z, p*w+r*yq-qrate*z, qrate*w-r*x+p*z, r*w+qrate*x-p*yq])
    out[10:13] = inertia_inv @ (moment - np.cross(omega, inertia @ omega))
    return out

def step(state: StateVector, force, moment, mass, inertia, inertia_inv, dt, scratch=None, lla0=None):
    y0 = state._y.copy(); h = float(dt)
    k1 = _rhs(y0, force, moment, mass, inertia, inertia_inv)
    k2 = _rhs(y0 + 0.5*h*k1, force, moment, mass, inertia, inertia_inv)
    k3 = _rhs(y0 + 0.5*h*k2, force, moment, mass, inertia, inertia_inv)
    k4 = _rhs(y0 + h*k3, force, moment, mass, inertia, inertia_inv)
    state.update_from_y(y0 + h*(k1 + 2*k2 + 2*k3 + k4)/6.0, lla0=lla0)
    state.accel_bb[:] = force / mass + np.cross(state.v_body, state.omega_body)
    state.accel_bi[:] = state.dcm_be.T @ (force / mass)
    state.domega_body[:] = _rhs(state._y, force, moment, mass, inertia, inertia_inv)[10:13]
    return state
