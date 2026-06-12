// Closed-loop behavior: the controller must drive a commanded slew into the
// deadband, hold it there, and never exceed the actuator limit.
#include <cmath>

#include "framework.hpp"
#include "slew/simulation.hpp"

using namespace slew;

SLEW_TEST(thirty_degree_slew_settles) {
  SimConfig cfg;  // defaults: 1 kHz, 20 s, kp/kd tuned for the default inertia
  cfg.q_cmd = default_slew_command(30.0);

  const SimResult r = run(cfg);

  CHECK(r.settle_time_s >= 0.0);          // it settled at all
  CHECK(r.settle_time_s < cfg.duration_s);
  CHECK(r.final_error_deg < cfg.deadband_deg);
  CHECK(r.peak_omega > 0.0);              // it actually moved
  CHECK(std::isfinite(r.final_error_deg));
}

SLEW_TEST(final_rate_is_near_zero) {
  SimConfig cfg;
  cfg.q_cmd = default_slew_command(45.0);
  const SimResult r = run(cfg);
  // A settled maneuver ends at rest, not coasting through the target.
  CHECK_NEAR(norm(r.trace.back().omega), 0.0, 1e-3);
}

SLEW_TEST(torque_never_exceeds_actuator_limit) {
  SimConfig cfg;
  cfg.tau_max = {0.5, 0.5, 0.5};
  cfg.q_cmd = default_slew_command(60.0);
  const SimResult r = run(cfg);
  // peak_torque is the max commanded component over the run.
  CHECK(r.peak_torque <= 0.5 + 1e-9);
}

SLEW_TEST(larger_slew_takes_at_least_as_long) {
  SimConfig small = SimConfig{};
  small.q_cmd = default_slew_command(10.0);
  SimConfig large = SimConfig{};
  large.q_cmd = default_slew_command(90.0);

  const SimResult rs = run(small);
  const SimResult rl = run(large);
  CHECK(rs.settle_time_s >= 0.0);
  CHECK(rl.settle_time_s >= 0.0);
  CHECK(rl.settle_time_s >= rs.settle_time_s);
}
