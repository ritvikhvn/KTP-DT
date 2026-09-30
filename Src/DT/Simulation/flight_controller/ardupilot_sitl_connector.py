"""ArduPilot SITL JSON physics-backend connector (UDP).

Implements ArduPilot's JSON SITL physics-backend protocol.

Protocol:
- SITL -> physics: binary
    uint16 magic (=18458), uint16 frame_rate, uint32 frame_count, uint16 pwm[16]
- physics -> SITL: plain-text JSON terminated by '\\n', fields:
    timestamp (s), imu:{gyro[3], accel_body[3]}, position[3], velocity[3],
    attitude[3] or quaternion[4]
Socket bound on UDP port 9002. Replies go to the sender address of the last recv.
"""
from __future__ import annotations

import json
import socket
import struct
import time
from dataclasses import dataclass

try:
    import orjson as _orjson
    _ORJSON = True
except ImportError:
    _orjson = None
    _ORJSON = False

import numpy as np

from ..state import StateVector
from ..constants import G_NED as _G_NED

try:
    import psutil as _psutil
    _PROC = _psutil.Process()
    _PROC.cpu_percent()   # first call seeds the counter; returns 0.0
    _HAVE_PSUTIL = True
except ImportError:
    _HAVE_PSUTIL = False


MAGIC_16 = 18458     # SERVO_32_ENABLE=0
MAGIC_32 = 29569     # SERVO_32_ENABLE=1


@dataclass
class RecvResult:
    pwm: np.ndarray      # (16,) or (32,)
    frame_rate: int
    frame_count: int
    reset: bool
    missed: int          # >0 if missed input frames


class SITLConnector:
    def __init__(self, port: int = 9002, print_every: int = 1000, verbose: bool = False, sim_rate: float = 400.0):
        self.port = port
        self.print_every = print_every
        self.verbose = verbose
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
        # Bounded block so SIGINT can land even if SITL goes silent — on Windows, Ctrl+C
        # does not interrupt an unbounded blocking recv, leaving the process hung forever.
        self._sock.settimeout(1.0)

        self._last_addr: tuple[str, int] | None = None
        self._last_frame: int = -1
        self._connected = False

        self._t_window = time.time()
        self._window_frames = 0
        self._last_sim_time = 0.0
        self._last_debug_print = 0.0
        self._tick_us_accum: float = 0.0   # cumulative tick compute time over window
        self._print_interval: float = 10.0  # wall-clock seconds between status lines
        self._sim_rate: float = sim_rate

    def close(self) -> None:
        self._sock.close()

    def recv_pwm(self) -> RecvResult:
        """Blocking recv of one SITL frame. Skips packets already superseded by newer buffered ones."""
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as e:
                raise RuntimeError(f"UDP recv failed: {e}") from e

            if len(data) < 8:
                continue
            magic = struct.unpack_from("<H", data, 0)[0]

            if magic == MAGIC_16:
                expected = 4 + 4 + 16 * 2
                n_pwm = 16
            elif magic == MAGIC_32:
                expected = 4 + 4 + 32 * 2
                n_pwm = 32
            else:
                print(f"[sitl] incorrect magic value {magic}")
                continue

            if len(data) != expected:
                # Buffer overrun or partial - skip
                continue

            frame_rate, frame_count = struct.unpack_from("<HI", data, 2)
            pwm = np.array(struct.unpack_from(f"<{n_pwm}H", data, 8), dtype=float)

            self._last_addr = addr

            reset = False
            missed = 0
            if frame_count < self._last_frame:
                # Controller restarted
                reset = True
                self._connected = False
                print("[sitl] Controller reset")
            elif frame_count == self._last_frame:
                # Duplicate frame - skip silently
                continue
            elif self._connected and frame_count != self._last_frame + 1:
                missed = frame_count - self._last_frame - 1
                print(f"[sitl] Missed {missed} input frames")

            self._last_frame = frame_count

            if not self._connected:
                self._connected = True
                print(f"[sitl] Connected to {addr[0]}:{addr[1]}")

            return RecvResult(pwm=pwm, frame_rate=int(frame_rate), frame_count=int(frame_count),
                              reset=reset, missed=missed)

    def report_progress(
        self,
        sim_time: float,
        pwm: np.ndarray | None = None,
        state: "StateVector | None" = None,
        tick_us: float = 0.0,
    ) -> None:
        """Print a periodic status line.

        Args:
            sim_time:  current simulation time [s]
            pwm:       last PWM array (printed in verbose mode)
            state:     optional state for diagnostics
            tick_us:   wall-clock µs spent inside sim.tick() for this frame

        Returns:
            A metrics dict when the status window fires (every `_print_interval`
            seconds), else None. The GUI publishes this dict as a status message.
        """
        self._window_frames += 1
        self._tick_us_accum += tick_us

        now = time.time()
        wall_dt = now - self._t_window
        if wall_dt < self._print_interval:
            return None
        sim_dt  = sim_time - self._last_sim_time
        fps     = self._window_frames / wall_dt if wall_dt > 0 else float("inf")
        ratio   = (sim_dt / wall_dt * 100.0)   if wall_dt > 0 else 0.0

        # Tick compute stats (physics only, excludes socket-wait time)
        avg_tick_us = self._tick_us_accum / self._window_frames
        target_us   = 1_000_000.0 / self._sim_rate
        budget_pct  = avg_tick_us / target_us * 100.0 if target_us > 0 else 0.0
        tick_str    = f"  tick={avg_tick_us:.0f}µs/{target_us:.0f}µs"

        # Process CPU% (psutil optional)
        cpu_pct = _PROC.cpu_percent() if _HAVE_PSUTIL else None
        cpu_str = f"  cpu={cpu_pct:.0f}%" if cpu_pct is not None else ""

        if self.verbose and pwm is not None:
            print(f"[sitl] {fps:.0f}fps  realtime(sim/wall)={ratio:.0f}%  t={sim_time:.2f}s"
                  f"{tick_str}{cpu_str}  frame={self._last_frame}"
                  f"  pwm={pwm.astype(int).tolist()}")
        else:
            print(f"[sitl] {fps:.0f}fps  realtime(sim/wall)={ratio:.0f}%  t={sim_time:.2f}s"
                  f"{tick_str}{cpu_str}")

        self._t_window      = now
        self._last_sim_time = sim_time
        self._window_frames = 0
        self._tick_us_accum = 0.0

        return {
            "fps": fps,
            "realtime_pct": ratio,
            "t": sim_time,
            "tick_us": avg_tick_us,
            "budget_pct": budget_pct,
            "cpu_pct": cpu_pct,
            "connected": self._connected,
        }

    def send_state(self, state: StateVector, timestamp: float) -> None:
        if self._last_addr is None:
            return
        accel = state.accel_bb - state.dcm_be @ _G_NED
        gyro  = state.omega_body
        payload = {
            "timestamp": float(timestamp),
            "imu": {"gyro": gyro, "accel_body": accel},
            "position": state.x_ned,
            "attitude": state.euler,
            "velocity": state.v_ned,
        }
        if _ORJSON:
            msg = b'\n' + _orjson.dumps(payload, option=_orjson.OPT_SERIALIZE_NUMPY) + b'\n'
        else:
            payload["imu"]["gyro"]         = payload["imu"]["gyro"].tolist()
            payload["imu"]["accel_body"]   = payload["imu"]["accel_body"].tolist()
            payload["position"]            = payload["position"].tolist()
            payload["attitude"]            = payload["attitude"].tolist()
            payload["velocity"]            = payload["velocity"].tolist()
            msg = ('\n' + json.dumps(payload, separators=(',', ':')) + '\n').encode('ascii')
        self._sock.sendto(msg, self._last_addr)
