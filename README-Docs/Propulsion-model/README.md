# Propulsion model

> [Interactive map](../system-architecture.html): *Propulsion model* (DC motor, BEMT single rotor: geometry, polars, solver). Download and open in a browser.

- Init: `git submodule update --init --recursive`.
- Package: `pip install -e Src/Propulsion-model`.
- Single rotor: `get_blade_geometry()` looks up matching APC `.PE0`; `solve_single_rotor()` runs isolated rotor analysis.
- No match: raises `NotImplementedError`; provide blade data or use matching APC geometry.
- `neuralfoil` / `aerosandbox`: optional; without them, thin-airfoil fallback supplies polars.
- DC motor: `solve_motor_equilibrium(..., load_torque_Nm=...)`; returns RPM, current, torque, power, efficiency.
- Motor and propeller models deliberately separate. No coupling solver ships.
- **Not used by the digital twin.** The twin's motor thrust/torque come from the curves in the [motor database](../database-doc.md); this package feeds the GUI Propulsion Model tab (`POST /api/propulsion/motor`, `/propeller`) and the CLIs only.
- Geometry match: the closest APC `.PE0` file by `diameter×pitch` in the file name, accepted if combined relative error ≤ 6 %; chord resampled to 20 stations, twist from `atan(P / (2π r))`.
- Polars: NeuralFoil Cl/Cd over α −5…25° (61 points) and Re 10 k…1 M, cached in memory and on disk (`data/polars_cache/`, git-ignored). Without NeuralFoil: thin-airfoil fallback with a warning — the airfoil name then has little effect.
- GUI motor sweep: the requested point plus 19 throttle values 0.1…1.0.
- Tests: `pytest Src/Propulsion-model/tests` (motor equilibrium is physical, APC 10×4.5 single-rotor smoke).

## CLI

Both models run standalone from the command line, same math as the GUI's Propulsion Model tab.

```sh
# DC motor — from Src/Propulsion-model
python aerodynamic_model/motor.py --kv 920 --rm 0.115 --i0 0.8 --v-batt 14.8 --throttle 0.7 --load-torque-nm 0.05
python aerodynamic_model/motor.py --help   # full flag list (eta-esc, k-iron, t-winding-c, ...)

# BEMT single rotor — from Src/Propulsion-model
python analysis_scripts/single_rotor_analysis.py --diameter-in 10 --pitch-in 4.5 --rpm 6000 --airfoil clark_y
python analysis_scripts/single_rotor_analysis.py --rpm 6000 --sweep-var V_CLIMB --sweep-values 0,2,4,6,8 --no-plots
python analysis_scripts/single_rotor_analysis.py --help
```

All flags optional — omitted ones fall back to the script's built-in defaults. `--no-plots` prints the summary table only (useful headless/CI). `--sweep-var`/`--sweep-values` overlay a parametric sweep on one shared plot instead of a single point.

## Theory — DC motor

Standard brushed/BLDC equivalent circuit, solved at equilibrium for a fixed external load torque (bench/dynamometer style — not derived from a propeller).

```
Kv_si = Kv * 2*pi/60                          # rpm/V -> rad/s/V
Rm_eff = Rm * (1 + 0.004*(T_winding_C - 20))  # copper resistance temp correction
V_eff = eta_ESC * throttle * V_batt

omega = (V_eff/Rm_eff - I0 - load_torque*Kv_si) / (1/(Kv_si*Rm_eff) + k_iron)
current = max((V_eff - omega/Kv_si) / Rm_eff, I0 + k_iron*omega)
torque = (current - I0 - k_iron*omega) / Kv_si

P_shaft = torque * omega
P_elec  = V_eff * current
eta_motor = P_shaft / P_elec
```

`k_iron` models iron/eddy losses as a speed-dependent current draw. `eta_ESC` is a flat throttle-scaling efficiency, not a switching-loss model.

## Theory — BEMT single rotor

Blade Element Momentum Theory, per-station solve (open-source `pyBEMT` algorithm, upstream equations unmodified):

- Split blade into annuli; each station has chord, twist, radius from APC geometry.
- Per station: solve inflow angle `phi` so momentum theory and blade-element force balance agree (`optimize.bisect`, fallback brute-force scan).
- Axial/tangential induction factors `a`, `a'` from Prandtl tip-loss-corrected momentum balance.
- Section lift/drag (`Cl`, `Cd`) from airfoil polars (NeuralFoil-computed, or bundled pyBEMT table) at local angle of attack and Reynolds number.
- Integrate `dT`, `dQ` over span → total thrust `T`, torque `Q`, aero power `P = Q*omega`.
- Non-dimensional coefficients: `CT = T/(rho*n^2*D^4)`, `CQ = Q/(rho*n^2*D^5)`, `CP = 2*pi*CQ`.

Radial arrays (`dT/dr`, `dQ/dr`, AoA, tip-loss `F`) available via `return_radial=True` for plotting.

