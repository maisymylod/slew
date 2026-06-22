// CSV output schema: a header row, one row per sample, 12 finite columns each.
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "framework.hpp"
#include "slew/simulation.hpp"

using namespace slew;

namespace {
std::vector<std::string> split(const std::string& line, char delim) {
  std::vector<std::string> out;
  std::string field;
  std::istringstream ss(line);
  while (std::getline(ss, field, delim)) out.push_back(field);
  return out;
}
}  // namespace

SLEW_TEST(csv_schema_and_finite_values) {
  SimConfig cfg;
  cfg.duration_s = 0.05;  // a handful of samples at the default 1000 Hz
  cfg.q_cmd = default_slew_command(20.0);
  const SimResult r = run(cfg);

  std::ostringstream out;
  write_csv(out, r);

  std::vector<std::string> lines;
  std::istringstream in(out.str());
  for (std::string line; std::getline(in, line);) lines.push_back(line);

  // Header present, then exactly one row per telemetry sample.
  CHECK(!lines.empty());
  CHECK(lines.front() == std::string(kCsvHeader));
  CHECK(lines.size() == r.trace.size() + 1);
  CHECK(split(kCsvHeader, ',').size() == 12);

  // Every data row has 12 columns and no NaN or infinity leaks through.
  for (std::size_t i = 1; i < lines.size(); ++i) {
    const std::vector<std::string> fields = split(lines[i], ',');
    CHECK(fields.size() == 12);
    for (const std::string& f : fields) {
      CHECK(std::isfinite(std::stod(f)));
    }
  }
}
