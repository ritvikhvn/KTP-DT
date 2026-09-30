"""CSV state logger."""
from __future__ import annotations
import csv
from pathlib import Path

class StateLogger:
    def __init__(self, path, rotor_channels=None):
        self.path = Path(path); self.path.parent.mkdir(parents=True, exist_ok=True); self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file); self._writer.writerow(["t", "x_n", "x_e", "x_d", "u", "v", "w", "roll", "pitch", "yaw", "p", "q", "r", "realtime_pct"]); self._file.flush()
    def write(self, t, state, pwm=None, realtime_pct=0.0):
        self._writer.writerow([t, *state.x_ned, *state.v_body, *state.euler, *state.omega_body, realtime_pct]); self._file.flush()
    def close(self):
        if not self._file.closed: self._file.close()
