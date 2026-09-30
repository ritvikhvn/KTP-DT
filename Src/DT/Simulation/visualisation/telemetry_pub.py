"""Optional ZMQ PUB telemetry stream for the standalone web GUI.

OFF by default. Enabled only when `startup_config.telemetry.enabled` is true or
the env var `DT_TELEMETRY_ENABLED=1` is set (the GUI launcher sets the latter).

Design notes:
- SNDHWM=1 + NOBLOCK send → a slow/dead subscriber can never stall the tick loop.
- Decimated to `rate_hz` (default 50 Hz), independent of the 1000 Hz sim rate.
- Port 9010 for telemetry.
"""
from __future__ import annotations

import os


class TelemetryPublisher:
    """Decimated, non-blocking ZMQ PUB of per-tick telemetry snapshots."""

    def __init__(self, endpoint: str, rate_hz: float = 50.0) -> None:
        import zmq

        self._zmq = zmq
        ctx = zmq.Context.instance()
        self._sock = ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 1)  # drop instead of blocking the tick
        # endpoint looks like tcp://127.0.0.1:9010 → bind on all interfaces by port
        port = endpoint.rsplit(":", 1)[-1]
        self._sock.bind(f"tcp://*:{port}")
        self._period = 1.0 / max(float(rate_hz), 1e-6)
        self._t_last = -1e18
        self.endpoint = endpoint
        self.rate_hz = float(rate_hz)

    def maybe_send(self, t: float, payload: dict) -> None:
        """Send payload if at least one decimation period has elapsed (sim time)."""
        if (t - self._t_last) < self._period:
            return
        self._t_last = t
        self.send(payload)

    def send(self, payload: dict) -> None:
        """Send immediately, no decimation (for low-rate messages e.g. status)."""
        try:
            self._sock.send_json(payload, flags=self._zmq.NOBLOCK)
        except self._zmq.ZMQError:
            pass  # HWM hit / no subscriber — drop silently

    def close(self) -> None:
        try:
            self._sock.close(linger=0)
        except Exception:
            pass


def make_telemetry_publisher(startup_cfg: dict) -> "TelemetryPublisher | None":
    """Return a publisher if telemetry is enabled (config flag or env), else None."""
    cfg = startup_cfg.get("telemetry", {})
    enabled = bool(cfg.get("enabled", False)) or os.environ.get("DT_TELEMETRY_ENABLED") == "1"
    if not enabled:
        return None
    endpoint = cfg.get("endpoint", "tcp://127.0.0.1:9010")
    rate_hz = float(cfg.get("rate_hz", 50))
    try:
        pub = TelemetryPublisher(endpoint, rate_hz)
        print(f"[main] telemetry PUB on {endpoint} @ {rate_hz:.0f} Hz")
        return pub
    except Exception as e:  # zmq missing or bind failure → run without telemetry
        print(f"[main] telemetry disabled (init failed): {e}")
        return None
