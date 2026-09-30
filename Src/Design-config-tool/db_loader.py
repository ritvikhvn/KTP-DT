"""Database access facade used by flight-time calculations."""
from __future__ import annotations

try:
    from .db_api import get_batteries, get_motors
except ImportError:
    from db_api import get_batteries, get_motors


def load_data() -> tuple[list[dict], list[dict]]:
    return get_motors(), get_batteries()
