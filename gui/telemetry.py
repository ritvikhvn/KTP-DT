"""Telemetry relay: ZMQ SUB on :9010 → fan-out to connected WebSockets.

Subscribes to the backend's telemetry PUB (main.py, gated by DT_TELEMETRY_ENABLED)
and forwards each decimated snapshot to every browser connected to /ws/telemetry.

Uses a plain blocking SUB in a background thread (not zmq.asyncio) so it works on
Windows' Proactor event loop that uvicorn installs. Messages are marshalled onto
the server's event loop via run_coroutine_threadsafe.
"""
from __future__ import annotations

import asyncio
import threading

DEFAULT_ENDPOINT = "tcp://127.0.0.1:9010"


class TelemetryHub:
    """Background SUB thread + WebSocket fan-out."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT) -> None:
        self.endpoint = endpoint
        self._clients: set = set()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._loop = asyncio.get_event_loop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="telemetry-sub", daemon=True)
        self._thread.start()

    async def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def add_client(self, ws) -> None:
        self._clients.add(ws)

    def remove_client(self, ws) -> None:
        self._clients.discard(ws)

    # ---- background thread -------------------------------------------------

    def _run(self) -> None:
        import zmq

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 10)   # 200 ms headroom at 50 Hz
        sock.connect(self.endpoint)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        try:
            while not self._stop.is_set():
                if dict(poller.poll(timeout=30)).get(sock) == zmq.POLLIN:
                    # Drain to freshest frame — drop stale queued messages
                    payload = None
                    try:
                        while True:
                            payload = sock.recv_json(flags=zmq.NOBLOCK)
                    except Exception:
                        pass
                    if payload is not None:
                        self._dispatch(payload)
        finally:
            sock.close(linger=0)

    def _dispatch(self, payload: dict) -> None:
        loop = self._loop
        if loop is None or not self._clients:
            return
        clients = list(self._clients)
        asyncio.run_coroutine_threadsafe(self._fan_out(clients, payload), loop)

    async def _fan_out(self, clients: list, payload: dict) -> None:
        await asyncio.gather(
            *(self._safe_send(ws, payload) for ws in clients),
            return_exceptions=True,
        )

    async def _safe_send(self, ws, payload: dict) -> None:
        try:
            await ws.send_json(payload)
        except Exception:
            self._clients.discard(ws)
