"""StateVector - vehicle state for the 6-DOF sim.

Frame conventions (ArduPilot / aerospace):
- NED earth frame: +N, +E, +Down
- Body frame: +X forward, +Y right, +Z down
- Quaternion order: [w, x, y, z] (scalar first), consistent with ArduPilot JSON
- DCM_be: earth-to-body (R_b_e). v_body = DCM_be @ v_ned.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation  # startup-only (not hot path)


# ---------------------------------------------------------------------------
# Scipy-free quaternion math helpers
# ---------------------------------------------------------------------------

def _quat_to_dcm_eb(q: np.ndarray, out: np.ndarray | None = None) -> np.ndarray:
    """Quaternion [w,x,y,z] -> body-to-earth DCM (3x3).
    dcm_be = result.T (earth->body).
    Uses the standard 10-multiply formula; avoids scipy Rotation overhead.
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    x2, y2, z2 = x + x, y + y, z + z
    xx = x * x2; xy = x * y2; xz = x * z2
    yy = y * y2; yz = y * z2; zz = z * z2
    wx = w * x2; wy = w * y2; wz = w * z2

    if out is None:
        out = np.empty((3, 3), dtype=np.float64)
    out[0, 0] = 1.0 - (yy + zz)
    out[0, 1] = xy - wz
    out[0, 2] = xz + wy
    out[1, 0] = xy + wz
    out[1, 1] = 1.0 - (xx + zz)
    out[1, 2] = yz - wx
    out[2, 0] = xz - wy
    out[2, 1] = yz + wx
    out[2, 2] = 1.0 - (xx + yy)
    return out


def _quat_to_euler(q: np.ndarray, out: np.ndarray | None = None) -> np.ndarray:
    """Quaternion [w,x,y,z] -> euler [roll, pitch, yaw] (ZYX, radians).
    Direct atan2/asin formulas; avoids scipy Rotation.as_euler overhead.
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    # roll (phi)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (theta)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
    pitch = math.asin(sinp)
    # yaw (psi)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if out is None:
        out = np.empty(3, dtype=np.float64)
    out[0] = roll
    out[1] = pitch
    out[2] = yaw
    return out


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    """Return normalised copy; preserves [w,x,y,z] order."""
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _normalize_quat_inplace(q: np.ndarray) -> None:
    """Normalise q in place; no allocation."""
    n = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if n > 1e-12:
        inv = 1.0 / n
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv


# ---------------------------------------------------------------------------
# Pure-array core for update_from_y (JIT target)
# ---------------------------------------------------------------------------

def _update_state_core(
    y_new: np.ndarray,
    y: np.ndarray,
    dcm_be: np.ndarray,
    euler: np.ndarray,
    v_ned: np.ndarray,
) -> None:
    """Copy y_new → y, normalise quat, compute dcm_be/euler/v_ned in-place.

    Computes dcm_be directly (no intermediate body-to-earth buffer + transpose copy).
    v_ned = L_EB @ v_body is computed as column-access on dcm_be (= L_EB.T).
    All trig (atan2, asin) supported by @njit.
    """
    for i in range(13):
        y[i] = y_new[i]

    # Normalise quaternion y[6:10] in-place
    qw = y[6]; qx = y[7]; qy = y[8]; qz = y[9]
    n = (qw*qw + qx*qx + qy*qy + qz*qz) ** 0.5
    if n > 1e-12:
        inv = 1.0 / n
        qw *= inv; qx *= inv; qy *= inv; qz *= inv
        y[6] = qw; y[7] = qx; y[8] = qy; y[9] = qz

    # Compute dcm_be = L_EB.T directly (10-multiply formula, transposed)
    # L_EB[i,j] given by _quat_to_dcm_eb; dcm_be[i,j] = L_EB[j,i].
    x2 = qx + qx;  y2 = qy + qy;  z2 = qz + qz
    xx = qx * x2;  xy = qx * y2;  xz = qx * z2
    yy = qy * y2;  yz = qy * z2;  zz = qz * z2
    wx = qw * x2;  wy = qw * y2;  wz = qw * z2

    dcm_be[0, 0] = 1.0 - (yy + zz);  dcm_be[0, 1] = xy + wz;        dcm_be[0, 2] = xz - wy
    dcm_be[1, 0] = xy - wz;           dcm_be[1, 1] = 1.0 - (xx + zz); dcm_be[1, 2] = yz + wx
    dcm_be[2, 0] = xz + wy;           dcm_be[2, 1] = yz - wx;         dcm_be[2, 2] = 1.0 - (xx + yy)

    # v_ned = L_EB @ v_body = dcm_be.T @ v_body (column-dot form)
    ub = y[3]; vb = y[4]; wb = y[5]
    v_ned[0] = dcm_be[0, 0]*ub + dcm_be[1, 0]*vb + dcm_be[2, 0]*wb
    v_ned[1] = dcm_be[0, 1]*ub + dcm_be[1, 1]*vb + dcm_be[2, 1]*wb
    v_ned[2] = dcm_be[0, 2]*ub + dcm_be[1, 2]*vb + dcm_be[2, 2]*wb

    # Euler angles (ZYX: roll, pitch, yaw) — math.atan2/asin supported by @njit
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    euler[0] = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = max(-1.0, min(1.0, sinp))
    euler[1] = math.asin(sinp)
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    euler[2] = math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# StateVector
# ---------------------------------------------------------------------------

class StateVector:
    """13-float flat integration buffer with named views + eager derived fields.

    Integration state (self._y, contiguous float64[13]):
      [0:3]   x_ned
      [3:6]   v_body
      [6:10]  quat  (w,x,y,z)
      [10:13] omega_body

    Derived (preallocated, recomputed by sync_derived / update_from_y):
      dcm_be (3,3), euler (3,), v_ned (3,), lla (3,)

    Post-RK4 outputs (preallocated, assigned by sixdof_quaternion.step):
      accel_bb (3,), accel_bi (3,), domega_body (3,)

    """

    __slots__ = (
        "_y",
        "x_ned", "v_body", "quat", "omega_body",   # views into _y
        "dcm_be", "euler", "v_ned", "lla",          # derived buffers
        "accel_bb", "accel_bi", "domega_body",      # post-RK4 outputs
        "_dcm_eb_buf",                              # scratch body->earth (sync_derived only)
    )

    def __init__(self) -> None:
        self._y = np.zeros(13, dtype=np.float64)
        self._y[6] = 1.0  # quat w=1 (identity)

        # Named views — share memory with _y, zero-copy
        self.x_ned      = self._y[0:3]
        self.v_body     = self._y[3:6]
        self.quat       = self._y[6:10]
        self.omega_body = self._y[10:13]

        # Preallocated derived buffers
        self._dcm_eb_buf = np.eye(3, dtype=np.float64)  # body->earth (sync_derived only)
        self.dcm_be      = np.eye(3, dtype=np.float64)  # earth->body (exposed)
        self.euler       = np.zeros(3, dtype=np.float64)
        self.v_ned       = np.zeros(3, dtype=np.float64)
        self.lla         = np.zeros(3, dtype=np.float64)

        # Post-RK4 outputs
        self.accel_bb    = np.zeros(3, dtype=np.float64)
        self.accel_bi    = np.zeros(3, dtype=np.float64)
        self.domega_body = np.zeros(3, dtype=np.float64)

    # ------------------------------------------------------------------
    # Derived-field recomputation (startup-only, not hot path)
    # ------------------------------------------------------------------

    def sync_derived(self) -> None:
        """Recompute euler, dcm_be, v_ned from current _y. Called at startup/reset."""
        _normalize_quat_inplace(self.quat)
        _quat_to_dcm_eb(self.quat, out=self._dcm_eb_buf)
        np.copyto(self.dcm_be, self._dcm_eb_buf.T)
        _quat_to_euler(self.quat, out=self.euler)
        np.dot(self._dcm_eb_buf, self.v_body, out=self.v_ned)

    # ------------------------------------------------------------------
    # In-place update (pool-of-1, hot tick path — no new object allocated)
    # ------------------------------------------------------------------

    def update_from_y(self, y_new: np.ndarray, lla0: np.ndarray | None = None) -> None:
        """Copy 13-float RK4 result into _y (views update automatically), recompute derived."""
        _update_state_core(y_new, self._y, self.dcm_be, self.euler, self.v_ned)
        if lla0 is not None:
            _ned_to_lla_inplace(self.x_ned, lla0, out=self.lla)

    @classmethod
    def from_initial(cls, initial_cfg: dict) -> "StateVector":
        euler = np.asarray(initial_cfg["euler"], dtype=float)
        quat = _euler_to_quat_wxyz(euler)
        s = cls()
        s.x_ned[:]      = np.asarray(initial_cfg["pos_ned"], dtype=float)
        s.v_body[:]     = np.asarray(initial_cfg["v_body"],  dtype=float)
        s.quat[:]       = quat
        s.omega_body[:] = np.asarray(initial_cfg["ang_rates"], dtype=float)
        s.lla[:]        = np.asarray(initial_cfg["pos_lla"],  dtype=float)
        s.sync_derived()
        return s


# ---------------------------------------------------------------------------
# Conversion helpers (startup-only or test use; not on hot tick path)
# ---------------------------------------------------------------------------

def _wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]])


def _xyzw_to_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[3], q[0], q[1], q[2]])


def _euler_to_quat_wxyz(euler: np.ndarray) -> np.ndarray:
    """[roll, pitch, yaw] -> quat [w,x,y,z]. ZYX intrinsic. Startup-only."""
    r = Rotation.from_euler("ZYX", [euler[2], euler[1], euler[0]], degrees=False)
    return _xyzw_to_wxyz(r.as_quat())


# ---------------------------------------------------------------------------
# LLA helpers
# ---------------------------------------------------------------------------

_EARTH_R_N = 6378137.0
_EARTH_R_E = 6378137.0


def _ned_to_lla(x_ned: np.ndarray, lla0: np.ndarray) -> np.ndarray:
    """Flat-earth NED -> LLA. lla0 = [lat_deg, lon_deg, alt_m]."""
    out = np.empty(3, dtype=np.float64)
    _ned_to_lla_inplace(x_ned, lla0, out)
    return out


def _ned_to_lla_inplace(x_ned: np.ndarray, lla0: np.ndarray, out: np.ndarray) -> None:
    lat0 = math.radians(lla0[0])
    d_lat = x_ned[0] / _EARTH_R_N
    d_lon = x_ned[1] / (_EARTH_R_E * math.cos(lat0))
    out[0] = lla0[0] + math.degrees(d_lat)
    out[1] = lla0[1] + math.degrees(d_lon)
    out[2] = lla0[2] - x_ned[2]
