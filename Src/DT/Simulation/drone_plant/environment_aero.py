"""Translational and rotational aerodynamic drag."""
from __future__ import annotations
import numpy as np

_f_drag_s = np.zeros(3); _m_drag_s = np.zeros(3); _v_rel_s = np.zeros(3); _f_gravity_s = np.zeros(3)

def aero_drag(v_ned, omega_body, dcm_be, v_gust_ned, rho, cd, surface_area_xyz, cd_moment, mass, g_ned):
    rel = dcm_be @ (v_ned - v_gust_ned)
    _v_rel_s[:] = rel
    _f_drag_s[:] = -0.5 * rho * surface_area_xyz * cd * rel * np.abs(rel)
    _m_drag_s[:] = -0.5 * rho * surface_area_xyz * cd_moment * omega_body * np.abs(omega_body)
    _f_gravity_s[:] = mass * (dcm_be @ g_ned)
    return _f_drag_s, _m_drag_s, _v_rel_s, _f_gravity_s
