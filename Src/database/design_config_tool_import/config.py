"""Repository-local database path."""
from __future__ import annotations

from pathlib import Path

_DATABASE_DIR = Path(__file__).resolve().parents[1]


def config() -> dict:
    return {"db_file": str(_DATABASE_DIR / "Motor_battery_database.sqlite")}
