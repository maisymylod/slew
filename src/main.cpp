// slew: command-line driver for the attitude-control simulator.
//
// Commands a slew maneuver, runs the fixed-rate control loop, and prints a
// summary (settle time, final pointing error, peak rate and torque). With
// --realtime it paces the loop against a steady clock and also reports timing
// jitter. With --csv it writes the full telemetry trace.
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "slew/simulation.hpp"

namespace {

using slew::Vec3;

void print_usage() {
  std::cout <<
      "slew - 3-DOF spacecraft attitude-control simulator\n\n"
      "Usage: slew [options]\n\n"
      "  --slew-deg N       commanded slew angle in degrees (default 30)\n"
      "  --rate-hz N        control loop frequency (default 1000)\n"
      "  --duration-s N     simulated duration in seconds (default 20)\n"
      "  --kp N             proportional gain (default 10.0)\n"
      "  --kd N             derivative gain (default 13.0)\n"
      "  --tau-max N        per-axis torque limit, N m (default 1.0)\n"
      "  --inertia X,Y,Z    principal moments of inertia (default 10,12,8)\n"
      "  --deadband-deg N   settle threshold in degrees (default 0.5)\n"
      "  --realtime         pace the loop against a steady clock, report jitter\n"
      "  --csv PATH         write the telemetry trace as CSV\n"
      "  --help             show this message\n";
}

// Minimal flag parsing: returns the value following `flag`, or `fallback`.
const char* opt(int argc, char** argv, const char* flag, const char* fallback) {
  for (int i = 1; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], flag) == 0) return argv[i + 1];
  }
  return fallback;
}
bool has_flag(int argc, char** argv, const char* flag) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], flag) == 0) return true;
  }
  return false;
}

Vec3 parse_vec3(const std::string& s, const Vec3& fallback) {
  Vec3 v = fallback;
  const int n = std::sscanf(s.c_str(), "%lf,%lf,%lf", &v.x, &v.y, &v.z);
  return n == 3 ? v : fallback;
}

void write_csv(const std::string& path, const slew::SimResult& r) {
  std::ofstream out(path);
  slew::write_csv(out, r);
}

}  // namespace

int main(int argc, char** argv) {
  if (has_flag(argc, argv, "--help")) {
    print_usage();
    return 0;
  }

  slew::SimConfig cfg;
  const double slew_deg = std::atof(opt(argc, argv, "--slew-deg", "30"));
  cfg.rate_hz = std::atof(opt(argc, argv, "--rate-hz", "1000"));
  cfg.duration_s = std::atof(opt(argc, argv, "--duration-s", "20"));
  cfg.kp = std::atof(opt(argc, argv, "--kp", "10.0"));
  cfg.kd = std::atof(opt(argc, argv, "--kd", "13.0"));
  const double tau_max = std::atof(opt(argc, argv, "--tau-max", "1.0"));
  cfg.tau_max = {tau_max, tau_max, tau_max};
  cfg.inertia = parse_vec3(opt(argc, argv, "--inertia", "10,12,8"), cfg.inertia);
  cfg.deadband_deg = std::atof(opt(argc, argv, "--deadband-deg", "0.5"));
  cfg.realtime = has_flag(argc, argv, "--realtime");
  cfg.q_cmd = slew::default_slew_command(slew_deg);

  if (std::string err = slew::validate_config(cfg); !err.empty()) {
    std::cerr << "slew: invalid configuration: " << err << '\n';
    return 2;
  }

  const slew::SimResult r = slew::run(cfg);

  std::cout.setf(std::ios::fixed);
  std::cout.precision(4);
  std::cout << "slew maneuver: " << slew_deg << " deg about (0.4, 0.7, 0.5)\n";
  std::cout << "control rate : " << cfg.rate_hz << " Hz over " << cfg.duration_s
            << " s (" << r.trace.size() << " samples)\n";
  std::cout << "inertia      : " << cfg.inertia.x << ", " << cfg.inertia.y
            << ", " << cfg.inertia.z << " kg m^2\n";
  std::cout << "gains        : kp=" << cfg.kp << " kd=" << cfg.kd
            << " tau_max=" << tau_max << " N m\n";
  std::cout << "----\n";
  if (r.settle_time_s >= 0.0) {
    std::cout << "settle time  : " << r.settle_time_s << " s (to within "
              << cfg.deadband_deg << " deg)\n";
  } else {
    std::cout << "settle time  : not settled within " << cfg.duration_s << " s\n";
  }
  std::cout << "final error  : " << r.final_error_deg << " deg\n";
  std::cout << "peak rate    : " << r.peak_omega << " rad/s\n";
  std::cout << "peak torque  : " << r.peak_torque << " N m\n";

  if (r.realtime) {
    std::cout << "----\n";
    std::cout.precision(2);
    std::cout << "timing jitter: mean " << r.jitter_mean_us << " us, p99 "
              << r.jitter_p99_us << " us, max " << r.jitter_max_us << " us\n";
  }

  const char* csv = opt(argc, argv, "--csv", "");
  if (csv[0] != '\0') {
    write_csv(csv, r);
    std::cout << "trace written: " << csv << '\n';
  }
  return 0;
}
