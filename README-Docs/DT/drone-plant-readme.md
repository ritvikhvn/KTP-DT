# Physics notes

> Interactive version: [system-architecture.html](../system-architecture.html) — open *Digital-twin backend ▸ Drone plant* (download the file and open it in a browser).

## Per-tick data flow

`Simulation.tick(pwm, dt)` is called once per ArduPilot frame ([DT README](README.md)). Inputs: `pwm[16]` (µs), `dt`. State held between ticks: the `StateVector`. Stages run in this order:

| # | Stage | In | Out |
|---|---|---|---|
| 1 | PWM → throttle (`pwm_to_throttle`) | `pwm[pwm_ch−1]`, `pwm_min/max` (1000/2000) | throttle 0…1 per rotor |
| 2 | Motor lag (`MotorDynamics.step`) | throttle, `dt` | lagged throttle (pass-through unless `actuator.enabled`) |
| 3 | Motor forces (`compute_motor_forces`) | throttle, rotor arms, `spin_dir`, database curve | `F_motors`, `M_motors` (body axes) |
| 4 | Ground contact (`ground_forces`) | `x_ned`, `v_ned`, `dcm_be`, ω | `F_ground`, `M_ground`, `on_ground` |
| 5 | Dryden gust (`DrydenTurbulence.update`) — only if airborne and enabled | altitude, \|v_body\|, `dcm_be` | `v_gust_ned` (else zeros) |
| 6 | Drag + gravity (`aero_drag`) | `v_ned`, ω, `dcm_be`, gust | `F_drag`, `M_drag`, `F_gravity` |
| 7 | Sum | all of the above | `F_total`, `M_total` |
| 8 | 6-DOF RK4 (`sixdof_quaternion.step`) | `y₀` (13 states), `F_total`, `M_total`, `dt` | new `y`, then derived and post-step outputs |

Outputs (the `StateVector`): the 13 integrated values `y = [x_ned, v_body, quat, ω_body]`; derived `dcm_be`, `euler`, `v_ned`, `lla`; post-step `accel_bb`, `accel_bi`, `domega_body`. The main loop reads them to build the ArduPilot reply, telemetry, viz frame and CSV row.

Notes: stage 4 runs before stage 5 because `on_ground` gates the gust; ground contact and drag write into module-level scratch arrays, so only one `Simulation` can exist per process; forces are held constant over the four RK4 stages.

## Summary

- State: NED position, body velocity, quaternion, body rates.
- Integrator: fixed-step fourth-order Runge-Kutta.
- Rotor output: thrust and reaction moment from database curves.
- Environment: quadratic drag and configurable Dryden gust (see [dryden-turbulence-readme.md](dryden-turbulence-readme.md)).
- Contact: vertical spring/damper plus bounded lateral friction.
- Battery energy accounting is outside runtime physics.

## Frames

| Frame | Axes |
|---|---|
| NED earth | +X North, +Y East, +Z Down |
| Body | +X forward, +Y right, +Z down |
| DCM `dcm_be` | Earth→body: `v_body = dcm_be @ v_ned` |
| Quaternion | `[w, x, y, z]` scalar-first |
| Thrust | Along −Z body (opposes gravity) |

## 6DOF rigid-body equations of motion

State vector, 13 elements:

```
y = [x_ned(3), v_body(3), quat_wxyz(4), omega_body(3)]
```

Position (NED), rotated from body velocity by the body→earth DCM:

```
ẋ_ned = dcm_be.T @ v_body
```

Translational dynamics, body frame (Coriolis term from rotating frame; gravity folded into `F_b`):

```
v̇_body = F_b / m − ω × v_body
```

Quaternion kinematics:

```
q̇ = 0.5 × Ω(ω) @ q

Ω(p,q,r) = ⎡  0  −p  −q  −r ⎤
           ⎢  p   0   r  −q ⎥
           ⎢  q  −r   0   p ⎥
           ⎣  r   q  −p   0 ⎦
```

Angular acceleration, body frame (gyroscopic/precession coupling from `ω × Iω`):

```
ω̇ = I⁻¹ × (M_b − ω × (I @ ω))
```

`I⁻¹` precomputed once at startup from the body inertia tensor.

## RK4 integration scheme

Fixed-step, classical 4th-order Runge-Kutta over the 13-state vector `y`:

```
k1 = f(y)
k2 = f(y + 0.5·dt·k1)
k3 = f(y + 0.5·dt·k2)
k4 = f(y +     dt·k3)
y_new = y + (dt/6) × (k1 + 2k2 + 2k3 + k4)
```

Quaternion is renormalized after every RK4 stage to stop norm drift over long runs (equivalent to a Baumgarte-style constraint correction).

Derived, post-integration:

| Output | Formula |
|---|---|
| Specific-force accel (body) | `F_b/m + v_body × ω` |
| Specific-force accel (earth) | `dcm_be.T @ (F_b/m)` |
| Angular accel | `I⁻¹ × (M_b − ω × Iω)` |

## Motor force/moment mapping

Per-rotor throttle (0–1) drives thrust, reaction torque and electrical power from a lookup curve:

```
F_rotor = [0, 0, −thrust(throttle)]              [−Z body = up]
M_rotor = arm_pos × F_rotor + [0, 0, spin_dir × torque(throttle)]

f_body = Σ F_rotor
m_body = Σ M_rotor
```

`arm_pos × F_rotor` is the moment arm (cross product, CG-relative). `spin_dir` = ±1 for CW/CCW rotors, giving the reaction-torque sign about body Z.

Rotor spin-up/down is not instantaneous — a first-order lag on commanded throttle models finite motor/ESC response:

```
throttle_eff[n] = a·throttle_eff[n-1] + (1-a)·throttle_cmd,   a = exp(−dt/τ)
```

`τ` = `actuator.motor_tau_s` in `startup_config.json` (0.07 s). The lag is **disabled by default** (`actuator.enabled=false`); when off, throttle passes straight through. Change it in the config and restart — nothing calls `MotorDynamics.set_param` at runtime.

## Aerodynamic drag and gravity

Relative wind (earth-frame gust subtracted, rotated to body):

```
v_rel_body = dcm_be @ (v_ned − v_gust_ned)
```

Quadratic drag, per body axis (`v·|v|` keeps opposing sign with quadratic magnitude):

```
f_drag[i] = −0.5 × ρ × S[i] × Cd[i] × v_rel_body[i] × |v_rel_body[i]|
m_drag[i] = −0.5 × ρ × S[i] × Cd_moment[i] × ω[i] × |ω[i]|
```

Gravity, resolved into body frame (only place gravity enters the model):

```
f_gravity_body = m × (dcm_be @ [0, 0, 9.80665])     # G in Simulation/constants.py
```

## Ground contact equations

Vertical (spring-damper). NED z is down; the ground plane is where `feet_z = z_down + cg_ground_offset` equals 0, and the drone is in contact when `feet_z ≥ 0` (`feet_z` = penetration depth):

```
F_z = min(0, -k_spring * feet_z - c_damp * v_z)      # negative = up in NED
```

At rest the drone settles about `m·g / k_spring` into the ground (≈ 2.9 cm for the 1.453 kg generic quad at 500 N/m). With the default `initial.pos_ned = [0,0,0]` the feet start 13 cm below the plane and are pushed up over the first frames.

Lateral friction, capped by Coulomb limit against vertical force:

```
F_lat = -k_lat_damp * v_lat
limit = mu_friction * |F_z|
F_lat = F_lat * min(1, limit / |F_lat|)
```

Rotational damping (all axes, applied only while in contact):

```
M = -k_rot_damp * omega_body
```

Params (`ground_contact` block in drone config): `k_spring`, `c_damp`, `mu_friction`, `k_lat_damp`, `k_rot_damp`.

## IMU signal sent to ArduPilot

`SITLConnector.send_state` builds the IMU from the plant outputs:

```
accel_bb   = F_total/m + v_body × ω            # post-RK4, gravity included (≈ 0 in steady hover)
accel_body = accel_bb − dcm_be @ g_ned         # what ArduPilot receives (≈ −g on Z in hover)
gyro       = ω_body
```

Position, velocity (NED) and Euler attitude go in the same JSON line. **Review note:** an ideal accelerometer measures specific force, `F_non-gravity/m`, which has no `ω×v` term; the expression above carries `v_body × ω` from `accel_bb`, so the IMU differs by that term whenever the drone translates and rotates at once. Confirm this is intended before comparing with flight data. The Live Data "Accel" plot shows `accel_bb`, not `accel_body`.
