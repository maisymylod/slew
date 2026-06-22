// Fixed-timestep attitude-control simulation: wires the plant (RigidBody) to
// the controller (QuaternionPD) and runs them at a fixed control rate.
//
// In the default (fast) mode the loop reads no clock and draws no randomness,
// so a given SimConfig produces a bit-identical trace every run. The optional
// real-time mode paces the loop against a steady clock and records the timing
// jitter; it does not change the computed trajectory.
#ifndef SLEW_SIMULATION_HPP
#define SLEW_SIMULATION_HPP

#include <string>
#include <vector>

#include "slew/quaternion.hpp"
#include "slew/rigid_body.hpp"
#include "slew/vec3.hpp"

namespace slew {

struct SimConfig {
  Vec3 inertia{10.0, 12.0, 8.0};   // principal moments of inertia (kg m^2)
  Quaternion q0{};                 // initial attitude (identity)
  Vec3 omega0{};                   // initial body rate (rad/s)
  Quaternion q_cmd{};              // commanded attitude

  double kp = 10.0;                // proportional gain
  double kd = 13.0;                // derivative gain
  Vec3 tau_max{1.0, 1.0, 1.0};     // per-axis actuator torque limit (N m)

  double rate_hz = 1000.0;         // control frequency
  double duration_s = 20.0;        // total simulated time
  double deadband_deg = 0.5;       // pointing-accuracy threshold for "settled"

  bool realtime = false;           // pace the loop against a steady clock
};

struct TelemetrySample {
  double t;
  Quaternion q;
  Vec3 omega;
  Vec3 tau;
  double error_deg;
};

struct SimResult {
  std::vector<TelemetrySample> trace;
  double settle_time_s = -1.0;     // first time error stays below deadband; -1 if never
  double final_error_deg = 0.0;
  double peak_omega = 0.0;         // max body rate reached (rad/s)
  double peak_torque = 0.0;        // max torque component commanded (N m)

  bool realtime = false;
  double jitter_mean_us = 0.0;
  double jitter_max_us = 0.0;
  double jitter_p99_us = 0.0;
};

// Check a SimConfig for physically meaningful values before running. Returns an
// empty string when the config is valid, otherwise a human-readable message
// naming the first offending field. The simulation assumes positive rate,
// duration, inertia, and torque limits; a non-positive or non-finite value
// (e.g. from `--rate-hz -100`) otherwise produces silent garbage.
std::string validate_config(const SimConfig& config);

SimResult run(const SimConfig& config);

// Commanded attitude for a slew of `degrees` about a fixed, deliberately
// off-axis vector (exercises all three control axes and the gyroscopic
// coupling between them).
Quaternion default_slew_command(double degrees);

}  // namespace slew

#endif  // SLEW_SIMULATION_HPP
