"""Entry point for the standalone web GUI:  python -m gui [--port 8080] [--no-browser]

This is the ONLY way the GUI starts.
"""
from __future__ import annotations

import argparse
import threading
import webbrowser


def main() -> int:
    p = argparse.ArgumentParser(prog="python -m gui", description="Digital-Twin web GUI")
    p.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080)")
    p.add_argument("--host", default="127.0.0.1", help="bind host (default 127.0.0.1)")
    p.add_argument("--no-browser", action="store_true", help="do not auto-open the browser")
    args = p.parse_args()

    import uvicorn

    from .launcher import _port_free
    from .server import create_app

    if not _port_free(args.port):
        print(f"[gui] port {args.port} is already in use — is another `python -m gui` still running? "
              f"(Ctrl+C that one, or pass --port to use a different one)")
        return 1

    url = f"http://{args.host if args.host != '0.0.0.0' else '127.0.0.1'}:{args.port}"
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    print(f"[gui] serving on {url}")
    uvicorn.run(create_app(), host=args.host, port=args.port, log_level="warning", access_log=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
