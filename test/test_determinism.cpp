// The core guarantee: in fast mode the engine reads no clock and draws no
// randomness, so the same configuration yields a bit-identical trajectory.
// This is what makes a recorded trace a trustworthy regression baseline.
#include "framework.hpp"
#include "slew/simulation.hpp"

using namespace slew;

namespace {
bool bit_identical(const TelemetrySample& a, const TelemetrySample& b) {
  return a.t == b.t && a.q.w == b.q.w && a.q.x == b.q.x && a.q.y == b.q.y &&
         a.q.z == b.q.z && a.omega.x == b.omega.x && a.omega.y == b.omega.y &&
         a.omega.z == b.omega.z && a.tau.x == b.tau.x && a.tau.y == b.tau.y &&
         a.tau.z == b.tau.z && a.error_deg == b.error_deg;
}
}  // namespace

SLEW_TEST(repeated_runs_are_bit_identical) {
  SimConfig cfg;
  cfg.q_cmd = default_slew_command(37.0);

  const SimResult a = run(cfg);
  const SimResult b = run(cfg);

  CHECK(a.trace.size() == b.trace.size());
  // Exact equality, not approximate: any hidden nondeterminism would show here.
  for (std::size_t i = 0; i < a.trace.size(); ++i) {
    CHECK(bit_identical(a.trace[i], b.trace[i]));
  }
  CHECK(a.settle_time_s == b.settle_time_s);
  CHECK(a.final_error_deg == b.final_error_deg);
}

SLEW_TEST(sample_count_matches_rate_and_duration) {
  SimConfig cfg;
  cfg.rate_hz = 500.0;
  cfg.duration_s = 4.0;
  cfg.q_cmd = default_slew_command(20.0);
  const SimResult r = run(cfg);
  CHECK(r.trace.size() == 2001);  // steps + 1 (inclusive of t=0 and t=duration)
}
