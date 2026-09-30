"""Shared physical constants.

Centralizes values that were previously duplicated across modules. Gravity is
standard gravity 9.80665 m/s² everywhere — body-frame rigid-body physics
(specific-force removal in the IMU model, gravity force in aero_drag, the SITL
connector's accel_body gravity re-injection), motor thrust gram-force→N, and the
barometric (ISA) pressure formula. This matches ArduPilot's GRAVITY_MSS so the
specific force on the wire is consistent with what AP expects.

CLAUDE.md rule 2/3: gravity is +Z (NED +down). `accel_bb` excludes gravity.
"""
from __future__ import annotations

import numpy as np

# Body-frame physics gravity (NED, +Z down).
G = 9.80665
G_NED = np.array([0.0, 0.0, G])

# ISA standard gravity — barometric formula only.
ISA_G = 9.80665
