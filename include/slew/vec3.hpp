// 3-vector with the handful of operations the dynamics and control need.
// Header-only, double precision, no allocation. Trivially copyable so it can
// live on the stack in the control loop's hot path.
#ifndef SLEW_VEC3_HPP
#define SLEW_VEC3_HPP

#include <cmath>

namespace slew {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3 operator-(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3 operator-(const Vec3& a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return s * a; }

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

inline Vec3 normalized(const Vec3& a) {
  const double n = norm(a);
  return n > 0.0 ? (1.0 / n) * a : Vec3{};
}

// Clamp each component to [-limit, +limit]. Used to model a saturating
// actuator (a reaction wheel or thruster bank cannot deliver unbounded torque).
inline Vec3 clamp_abs(const Vec3& a, const Vec3& limit) {
  auto c = [](double v, double l) { return v > l ? l : (v < -l ? -l : v); };
  return {c(a.x, limit.x), c(a.y, limit.y), c(a.z, limit.z)};
}

}  // namespace slew

#endif  // SLEW_VEC3_HPP
