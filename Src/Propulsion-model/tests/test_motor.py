import numpy as np

from aerodynamic_model.motor import solve_motor_equilibrium


def test_motor_equilibrium_is_physical():
    result = solve_motor_equilibrium(920, 0.115, 0.8, 14.8, 0.7, load_torque_Nm=0.05)
    assert np.isfinite(result["rpm"]) and result["rpm"] > 0
    assert result["current_A"] >= 0
    assert 0 <= result["eta_motor"] <= 1
