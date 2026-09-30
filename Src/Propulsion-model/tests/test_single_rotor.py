import numpy as np

from aerodynamic_model.geometry import get_blade_geometry
from aerodynamic_model.single_rotor import solve_single_rotor


def test_single_rotor_apc_smoke():
    geo = get_blade_geometry(10 * 0.0254, 4.5 * 0.0254, 2, 6000, 5)
    result = solve_single_rotor(geo, None, rpm=6000, V_climb=0)
    assert np.isfinite(result["T_N"]) and result["T_N"] > 0
    assert np.isfinite(result["Q_Nm"]) and result["Q_Nm"] > 0
