# Control law and gain tuning

The attitude controller (`include/slew/controller.hpp`) is a quaternion-feedback
PD law with per-axis torque saturation:

```
q_err = conj(q_cmd) * q          # rotation from commanded to current attitude
tau   = -kp * q_err.vec - kd * omega
tau   = clamp(tau, +/- tau_max)  # finite-authority actuator, per axis
```

`q_err` is forced onto the short path (scalar part non-negative) so the body
never slews the long way around. The defaults are `kp = 10`, `kd = 13`,
`tau_max = 1 N m`, with the default rigid body `inertia = (10, 12, 8) kg m^2`.

## Why these gains (small-angle analysis)

For a small attitude error, the vector part of the error quaternion is
`q_err.vec ~= theta / 2`, where `theta` is the rotation-vector error. Away from
saturation, and ignoring the (small, near rest) gyroscopic coupling, each body
axis reduces to an independent second-order system:

```
I * theta_ddot + kd * theta_dot + (kp / 2) * theta = 0
```

Matching `theta_ddot + 2*zeta*wn*theta_dot + wn^2*theta = 0` gives the natural
frequency and damping ratio per axis:

```
wn   = sqrt( (kp/2) / I )
zeta = kd / ( 2 * sqrt( (kp/2) * I ) )
```

With the default gains and inertia:

| Axis | I (kg m^2) | wn (rad/s) | zeta | approx settle 4/(zeta*wn) |
| --- | --- | --- | --- | --- |
| x | 10 | 0.707 | 0.919 | 6.2 s |
| y | 12 | 0.645 | 0.839 | 7.4 s |
| z | 8  | 0.791 | 1.028 | 4.9 s |

So the default gains place every axis near critical damping (`zeta ~ 0.84 to
1.03`): fast settling with little to no overshoot, which is what the closed-loop
test asserts (the body eases into the deadband and holds, rather than ringing).
The numbers above are design guides from the linearization, not exact settle
times; the simulator measures the real settle time.

## Large-angle behavior

For a large commanded slew the torque saturates at `tau_max`, so the initial
phase is actuator-rate limited (effectively bang-then-coast), not linear. The
analysis above governs the final approach once the error is small enough that
the torque comes out of saturation. Raising `tau_max` shortens the saturated
phase; it does not change the linear damping.

## Re-tuning for a different body

Pick a target natural frequency `wn` (how fast) and damping ratio `zeta` (how
much overshoot; `1.0` is critical, `0.7` is a common fast-but-slight-overshoot
choice), then invert the relations above per axis:

```
kp = 2 * I * wn^2
kd = 2 * I * zeta * wn
```

Notes:
- `kp`/`kd` here are scalars applied to all axes, so if the inertia is very
  anisotropic, choose `I` for the axis you care most about (or the largest, for
  the most conservative damping) and accept slightly different `wn`/`zeta` on the
  others, as the table shows for the default body.
- Keep `tau_max` consistent with the real actuator. Gains that demand more than
  `tau_max` simply saturate; they do not speed up the saturated phase.
- After changing gains, run the suite (`make test`) and a `--realtime` slew to
  confirm the body still settles into the deadband without overshoot.

## Reference

B. Wie, *Space Vehicle Dynamics and Control*, AIAA Education Series. The
quaternion-feedback PD form here follows the Wie/Lizarralde-style law referenced
in `controller.hpp`.
