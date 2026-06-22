# slew

[![CI](https://github.com/maisymylod/slew/actions/workflows/ci.yml/badge.svg)](https://github.com/maisymylod/slew/actions/workflows/ci.yml)

A 3-DOF spacecraft attitude-control simulator in C++20. One rigid body, a
quaternion-feedback controller, and a fixed-rate control loop that is
deterministic by construction: the same configuration produces a bit-identical
trajectory every run. It builds and tests with nothing but a C++20 compiler.

The point is the control problem done honestly: real rotational dynamics
(Euler's equations, not a small-angle shortcut), a controller that respects a
finite actuator, and a loop whose timing you can either ignore (fast,
reproducible) or measure (real-time, with jitter reported).

## What it does

A default run commands a 30-degree slew about an off-axis vector and reports how
the closed loop handles it:

```
$ slew --slew-deg 30
slew maneuver: 30.0000 deg about (0.4, 0.7, 0.5)
control rate : 1000.0000 Hz over 20.0000 s (20001 samples)
inertia      : 10.0000, 12.0000, 8.0000 kg m^2
gains        : kp=10.0000 kd=13.0000 tau_max=1.0000 N m
----
settle time  : 7.2460 s (to within 0.5000 deg)
final error  : 0.0008 deg
peak rate    : 0.1387 rad/s
peak torque  : 1.0000 N m
```

The torque saturates at the 1 N m actuator limit during the fast part of the
slew, then the controller eases the body into the deadband and holds it there.
`--csv trace.csv` writes the full per-step telemetry (attitude, body rate,
commanded torque, pointing error).

## The model

- **Plant** (`rigid_body.hpp`). Rigid-body rotational dynamics integrated with a
  fixed-step RK4: Euler's equation `J w_dot = tau - w x (J w)` for the body rate,
  and the quaternion kinematics `q_dot = 0.5 q (x) [0, w]` for the attitude. The
  attitude is renormalized onto the unit sphere after each step. Inertia is
  diagonal (principal axes); the gyroscopic coupling between axes is real, not
  dropped.
- **Controller** (`controller.hpp`). A quaternion-feedback PD law,
  `tau = -kp * q_err.vec - kd * omega`, where `q_err` is forced onto the short
  rotation path so the body never slews the long way around. The commanded
  torque is saturated per axis to model a finite-authority actuator.
- **Loop** (`simulation.cpp`). A fixed-timestep control loop at a configurable
  rate. In the default (fast) mode it reads no clock and draws no randomness, so
  the trajectory is reproducible to the bit. With `--realtime` it paces itself
  against a steady clock and reports the timing jitter (mean / p99 / max),
  without changing the computed trajectory. The hot path does no heap
  allocation; the telemetry buffer is reserved up front.

## Why deterministic matters

A trace is only a useful regression baseline if it is reproducible. The
determinism here is structural, not lucky: no wall-clock reads, no RNG, no
hash-ordered iteration on the fast path. The test suite asserts it directly by
running the same configuration twice and comparing every sample for exact
equality, so any nondeterminism introduced later fails the build.

## Build and run

With CMake (canonical):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/slew --slew-deg 45 --csv trace.csv
```

Or with just `make` and a compiler (no CMake needed):

```sh
make            # build ./bin/slew
make test       # build and run the suite
make asan       # run the suite under AddressSanitizer + UBSan
```

## Tests

Fourteen cases, all offline and deterministic (`test/`):

- **Quaternion algebra**: unit-norm invariants, Hamilton product, axis-angle
  round-trips, and that the attitude error always takes the short path
  (a 270-degree command reads back as 90 degrees the other way).
- **Plant physics**: a torque-free asymmetric body conserves angular-momentum
  magnitude and rotational kinetic energy across a 20-second tumble (to 1e-6); a
  symmetric body holds its spin vector exactly; a constant torque spins the body
  up at `tau / J`.
- **Closed loop**: a commanded slew settles into the deadband and ends at rest,
  the commanded torque never exceeds the actuator limit, and a larger slew never
  settles faster than a smaller one.
- **Determinism**: two runs of the same configuration are bit-identical, and the
  sample count matches the rate and duration exactly.

CI builds and tests on Linux and macOS, repeats the suite under
AddressSanitizer and UBSan, and checks formatting with clang-format.

## Scope and honesty

This is a faithful simulation of the attitude-control problem, not flight
software. It models one rigid body with ideal sensing and actuation: no sensor
noise, no actuator lag, no measurement delay, no flexible-body modes or fuel
slosh, and no momentum-storage or desaturation dynamics. Inertia is diagonal and
the PD gains are hand-tuned for the default body rather than derived for a
specification. The small-angle stability analysis behind the default gains, and
how to re-tune them for a different inertia, are written up in
[docs/CONTROL_LAW.md](docs/CONTROL_LAW.md). What it does get right is the rotational dynamics, the
short-path quaternion control with a saturating actuator, and a control loop
that is deterministic offline and measurable in real time. Those are the parts
worth getting right first.

## Layout

```
include/slew/vec3.hpp        3-vector ops
include/slew/quaternion.hpp  unit-quaternion attitude algebra
include/slew/rigid_body.hpp  Euler dynamics + RK4 integrator
include/slew/controller.hpp  quaternion-feedback PD with torque saturation
include/slew/simulation.hpp  config, telemetry, result types
src/simulation.cpp           the fixed-rate control loop
src/main.cpp                 CLI driver
test/                        dependency-free test suite
```

## Author

Maisy Mylod. B.S. Pure Mathematics, University of Michigan.
[GitHub](https://github.com/maisymylod) / [LinkedIn](https://linkedin.com/in/maisymylod)
