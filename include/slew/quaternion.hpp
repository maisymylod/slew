// Unit-quaternion attitude representation (Hamilton convention, scalar-first).
// A quaternion q maps body-frame vectors into the inertial frame. Header-only,
// double precision, no allocation.
#ifndef SLEW_QUATERNION_HPP
#define SLEW_QUATERNION_HPP

#include <cmath>

#include "slew/vec3.hpp"

namespace slew {

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec3 vec() const { return {x, y, z}; }
};

// Componentwise add and scale. These do not preserve unit norm; they exist so
// the RK4 integrator can form weighted combinations of state derivatives. The
// integrator renormalizes the attitude after each step.
inline Quaternion operator+(const Quaternion& a, const Quaternion& b) {
  return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Quaternion operator*(double s, const Quaternion& q) {
  return {s * q.w, s * q.x, s * q.y, s * q.z};
}
inline Quaternion operator-(const Quaternion& q) { return {-q.w, -q.x, -q.y, -q.z}; }

// Hamilton product.
inline Quaternion operator*(const Quaternion& a, const Quaternion& b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

inline Quaternion conjugate(const Quaternion& q) { return {q.w, -q.x, -q.y, -q.z}; }
inline double norm(const Quaternion& q) {
  return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

inline Quaternion normalized(const Quaternion& q) {
  const double n = norm(q);
  return n > 0.0 ? (1.0 / n) * q : Quaternion{};
}

// Rotation of `angle` radians about a unit `axis`.
inline Quaternion from_axis_angle(const Vec3& axis, double angle) {
  const Vec3 u = normalized(axis);
  const double h = 0.5 * angle;
  const double s = std::sin(h);
  return {std::cos(h), u.x * s, u.y * s, u.z * s};
}

// Smallest rotation angle (radians) between two attitudes, in [0, pi].
inline double angle_between(const Quaternion& a, const Quaternion& b) {
  const Quaternion e = conjugate(a) * b;  // rotation from a to b
  const double vec_norm = norm(e.vec());
  return 2.0 * std::atan2(vec_norm, std::fabs(e.w));
}

}  // namespace slew

#endif  // SLEW_QUATERNION_HPP
