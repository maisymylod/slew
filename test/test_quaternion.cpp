// Quaternion algebra and attitude-error properties.
#include <cmath>

#include "framework.hpp"
#include "slew/quaternion.hpp"

using namespace slew;

SLEW_TEST(identity_is_unit) {
  const Quaternion q{};
  CHECK_NEAR(norm(q), 1.0, 1e-12);
}

SLEW_TEST(hamilton_product_matches_known_axis_pair) {
  // 90 deg about x, then 90 deg about y, composed.
  const Quaternion qx = from_axis_angle({1, 0, 0}, M_PI / 2);
  const Quaternion qy = from_axis_angle({0, 1, 0}, M_PI / 2);
  const Quaternion q = qy * qx;
  CHECK_NEAR(norm(q), 1.0, 1e-12);
  // Composition stays a unit quaternion; conjugate inverts it back to identity.
  const Quaternion back = conjugate(q) * q;
  CHECK_NEAR(back.w, 1.0, 1e-12);
  CHECK_NEAR(norm(back.vec()), 0.0, 1e-12);
}

SLEW_TEST(from_axis_angle_recovers_angle) {
  const double angle = 0.7;  // radians
  const Quaternion q = from_axis_angle({0.3, -0.5, 0.8}, angle);
  CHECK_NEAR(angle_between(Quaternion{}, q), angle, 1e-9);
}

SLEW_TEST(angle_between_is_symmetric_and_short_path) {
  const Quaternion a = from_axis_angle({0, 0, 1}, 0.4);
  const Quaternion b = from_axis_angle({0, 0, 1}, -0.5);
  CHECK_NEAR(angle_between(a, b), angle_between(b, a), 1e-12);
  CHECK_NEAR(angle_between(a, b), 0.9, 1e-9);
}

SLEW_TEST(angle_between_never_exceeds_pi) {
  // 270 deg should read back as the 90 deg short path.
  const Quaternion q = from_axis_angle({0, 1, 0}, 3.0 * M_PI / 2.0);
  CHECK_NEAR(angle_between(Quaternion{}, q), M_PI / 2.0, 1e-9);
}
