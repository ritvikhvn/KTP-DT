# Dryden turbulence model

Discrete Dryden gust model, MIL-F-8785C spec (metric, low-altitude form). Linear gusts only (no angular/rotational gust — not needed for multirotor scale).

Where it sits: stage 5 of the [drone plant](drone-plant-readme.md#per-tick-data-flow) ([interactive map](../system-architecture.html): *Drone plant ▸ 5 · Dryden turbulence*). Its output `v_gust_ned` feeds the drag model (`v_rel = dcm_be·(v_ned − v_gust_ned)`) and is published in telemetry as `wind_ned`.

## Theory

Shaping filter, first-order, discrete:

```
x[n] = (1 - V/L * dt) * x[n-1] + sqrt(2*V/L * dt) * sigma * eta[n]
```

`V` = airspeed, `L` = turbulence scale length, `sigma` = turbulence intensity, `eta` = unit Gaussian noise, `dt` = filter step.

Scale lengths (altitude `h_ft` in feet):

```
Lw = h_ft
Lu = Lv = h_ft / (0.177 + 0.000823*h_ft)^1.2
```

Turbulence intensities (`W20` = wind speed at 20 ft / 6 m, m/s):

```
sigma_w = 0.1 * W20
sigma_u = sigma_v = sigma_w / (0.177 + 0.000823*h_ft)^0.4
```

Output: gust `u, v, w` in body frame → rotated to NED, added to mean wind vector (from heading + horizontal/vertical speed config).

## Config

- `W20_ms`: gust intensity driver.
- `wind_heading_deg`: compass heading of the mean wind. **Convention check:** the code builds the mean wind as `speed·[cos θ, sin θ, vertical]` in NED, i.e. the vector points *toward* θ (θ clockwise from North), although the original docstring says "from". Verify before comparing with real wind logs.
- `horizontal_wind_speed_ms`, `vertical_wind_speed_ms`: mean wind (steady component).
- `dt_filt`: filter update period, independent of physics tick `Ts`. Must be `>= Ts`. Filter holds last value between updates.
- `seeds`: per-channel RNG seeds (`[u, v, w, pg]`; `pg` unused). **Ignored when `sim.seed` is set** in `startup_config.json` (default `sim.seed = 1`): the three channels are then spawned from the master seed's Dryden child. `reset()` re-creates identical generators.
- `enabled`: `false` disables the whole block — **no mean wind either**, not just no gusts.

## Notes

- Below ground / near-zero altitude: floored at 0.5 m to avoid `Lw -> 0`.
- Airspeed floored at 0.5 m/s to avoid divide-by-zero in filter alpha.
- Gust disabled while vehicle on ground (`on_ground` from the ground-contact stage); the output is then zeros.
- `dt_filt` accumulates the fixed physics period `1/sim_rate` per call, not the actual frame `dt`.
- Runtime `set_wind` / `reset_wind_to_startup` exist on the class, but nothing in this repo sends wind commands; change config and restart.
