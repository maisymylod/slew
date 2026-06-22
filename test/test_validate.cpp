// CLI/config validation: invalid SimConfig values must be rejected, not run.
#include "framework.hpp"
#include "slew/simulation.hpp"

using namespace slew;

SLEW_TEST(default_config_is_valid) {
  CHECK(validate_config(SimConfig{}).empty());
}

SLEW_TEST(non_positive_rate_is_rejected) {
  SimConfig cfg;
  cfg.rate_hz = -100.0;
  const std::string err = validate_config(cfg);
  CHECK(!err.empty());
  CHECK(err.find("rate-hz") != std::string::npos);
}

SLEW_TEST(zero_duration_is_rejected) {
  SimConfig cfg;
  cfg.duration_s = 0.0;
  CHECK(!validate_config(cfg).empty());
}

SLEW_TEST(non_finite_inertia_is_rejected) {
  SimConfig cfg;
  cfg.inertia.y = std::nan("");
  const std::string err = validate_config(cfg);
  CHECK(!err.empty());
  CHECK(err.find("inertia") != std::string::npos);
}

SLEW_TEST(negative_tau_max_is_rejected) {
  SimConfig cfg;
  cfg.tau_max = {1.0, -1.0, 1.0};
  CHECK(!validate_config(cfg).empty());
}

SLEW_TEST(zero_deadband_is_allowed) {
  SimConfig cfg;
  cfg.deadband_deg = 0.0;  // non-negative is fine
  CHECK(validate_config(cfg).empty());
}

SLEW_TEST(negative_deadband_is_rejected) {
  SimConfig cfg;
  cfg.deadband_deg = -0.5;
  CHECK(!validate_config(cfg).empty());
}
