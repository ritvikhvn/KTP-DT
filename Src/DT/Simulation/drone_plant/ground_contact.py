"""Spring, damper, friction, and rotational damping at ground contact."""
from __future__ import annotations
import numpy as np

_f_gnd_s = np.zeros(3); _m_gnd_s = np.zeros(3)

def ground_forces(x_ned, v_ned, dcm_be, omega_body, cg_ground_offset, k_spring, c_damp, mu_friction, k_lat_damp, k_rot_damp):
    feet_z = float(x_ned[2] + cg_ground_offset)
    _f_gnd_s.fill(0); _m_gnd_s.fill(0)
    if feet_z < 0: return _f_gnd_s, _m_gnd_s, False
    vertical = min(0.0, -k_spring * feet_z - c_damp * v_ned[2])
    lateral = -k_lat_damp * np.asarray(v_ned[:2], dtype=float)
    limit = mu_friction * abs(vertical)
    norm = float(np.linalg.norm(lateral))
    if norm > limit and norm > 0: lateral *= limit / norm
    _f_gnd_s[:] = dcm_be @ np.array([lateral[0], lateral[1], vertical])
    _m_gnd_s[:] = -k_rot_damp * omega_body
    return _f_gnd_s, _m_gnd_s, True
