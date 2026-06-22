#include "slew/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ostream>
#include <thread>
#include <utility>

#include "slew/controller.hpp"

namespace slew {

namespace {
constexpr double kRadToDeg = 57.29577951308232;
}  // namespace

const char* const kCsvHeader =
    "t,qw,qx,qy,qz,wx,wy,wz,tau_x,tau_y,tau_z,error_deg";

void write_csv(std::ostream& out, const SimResult& result) {
  out << kCsvHeader << '\n';
  for (const auto& s : result.trace) {
    out << s.t << ',' << s.q.w << ',' << s.q.x << ',' << s.q.y << ',' << s.q.z
        << ',' << s.omega.x << ',' << s.omega.y << ',' << s.omega.z << ','
        << s.tau.x << ',' << s.tau.y << ',' << s.tau.z << ',' << s.error_deg
        << '\n';
  }
}

Quaternion default_slew_command(double degrees) {
  // A fixed off-axis rotation axis so the maneuver couples all three body axes.
  const Vec3 axis{0.4, 0.7, 0.5};
  return from_axis_angle(axis, degrees / kRadToDeg);
}

std::string validate_config(const SimConfig& config) {
  auto positive = [](const char* name, double v) -> std::string {
    if (!std::isfinite(v) || v <= 0.0) {
      return std::string(name) + " must be a finite positive number";
    }
    return {};
  };
  const std::pair<const char*, double> required[] = {
      {"rate-hz", config.rate_hz},     {"duration-s", config.duration_s},
      {"inertia.x", config.inertia.x}, {"inertia.y", config.inertia.y},
      {"inertia.z", config.inertia.z}, {"tau-max.x", config.tau_max.x},
      {"tau-max.y", config.tau_max.y}, {"tau-max.z", config.tau_max.z},
  };
  for (const auto& [name, value] : required) {
    if (std::string err = positive(name, value); !err.empty()) {
      return err;
    }
  }
  if (!std::isfinite(config.deadband_deg) || config.deadband_deg < 0.0) {
    return "deadband-deg must be a finite non-negative number";
  }
  return {};
}

SimResult run(const SimConfig& config) {
  const RigidBody body(config.inertia);
  const QuaternionPD controller(config.kp, config.kd, config.tau_max);

  const double dt = 1.0 / config.rate_hz;
  const int steps = static_cast<int>(std::lround(config.duration_s * config.rate_hz));

  SimResult result;
  result.realtime = config.realtime;
  result.trace.reserve(static_cast<std::size_t>(steps) + 1);

  // Real-time pacing state. Touched only when config.realtime is set, so the
  // fast path reads no clock and stays deterministic.
  std::vector<double> jitter_us;
  using clock = std::chrono::steady_clock;
  clock::time_point start{};
  if (config.realtime) {
    jitter_us.reserve(static_cast<std::size_t>(steps) + 1);
    start = clock::now();
  }

  AttitudeState state{config.q0, config.omega0};

  for (int i = 0; i <= steps; ++i) {
    const double t = i * dt;
    const Vec3 tau = controller.torque(state.q, state.omega, config.q_cmd);
    const double error_deg = angle_between(config.q_cmd, state.q) * kRadToDeg;

    result.trace.push_back({t, state.q, state.omega, tau, error_deg});

    result.peak_omega = std::max(result.peak_omega, norm(state.omega));
    result.peak_torque = std::max(
        {result.peak_torque, std::fabs(tau.x), std::fabs(tau.y), std::fabs(tau.z)});

    if (config.realtime) {
      const auto target = start + std::chrono::duration_cast<clock::duration>(
                                      std::chrono::duration<double>(t));
      const double err_us =
          std::chrono::duration<double, std::micro>(clock::now() - target).count();
      jitter_us.push_back(std::fabs(err_us));
      const auto next = start + std::chrono::duration_cast<clock::duration>(
                                    std::chrono::duration<double>((i + 1) * dt));
      std::this_thread::sleep_until(next);
    }

    if (i < steps) {
      state = body.step(state, tau, dt);
    }
  }

  result.final_error_deg = result.trace.back().error_deg;

  // Settle time: the first sample after which the error never again leaves the
  // deadband. Scan backwards for the last violation.
  result.settle_time_s = 0.0;
  for (int i = static_cast<int>(result.trace.size()) - 1; i >= 0; --i) {
    if (result.trace[i].error_deg > config.deadband_deg) {
      result.settle_time_s = (i + 1 < static_cast<int>(result.trace.size()))
                                 ? result.trace[i + 1].t
                                 : -1.0;  // still outside the deadband at the end
      break;
    }
  }

  if (config.realtime && !jitter_us.empty()) {
    double sum = 0.0;
    for (double j : jitter_us) sum += j;
    result.jitter_mean_us = sum / static_cast<double>(jitter_us.size());
    result.jitter_max_us = *std::max_element(jitter_us.begin(), jitter_us.end());
    std::sort(jitter_us.begin(), jitter_us.end());
    const std::size_t idx = static_cast<std::size_t>(
        std::min<double>(jitter_us.size() - 1, 0.99 * (jitter_us.size() - 1)));
    result.jitter_p99_us = jitter_us[idx];
  }

  return result;
}

}  // namespace slew
