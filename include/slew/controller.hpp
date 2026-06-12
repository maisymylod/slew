// Quaternion-feedback PD attitude controller (Wie/Lizarralde-style).
//
//   q_err = conj(q_cmd) * q          rotation from the commanded attitude
//   tau   = -kp * q_err.vec - kd * omega
//
// q_err is forced onto the short path (scalar part >= 0) so the spacecraft
// never slews the long way around. The commanded torque is saturated per axis
// to model a finite-authority actuator. The control law is allocation-free and
// runs entirely on the stack.
#ifndef SLEW_CONTROLLER_HPP
#define SLEW_CONTROLLER_HPP

#include "slew/quaternion.hpp"
#include "slew/vec3.hpp"

namespace slew {

class QuaternionPD {
 public:
  QuaternionPD(double kp, double kd, const Vec3& tau_max)
      : kp_(kp), kd_(kd), tau_max_(tau_max) {}

  Vec3 torque(const Quaternion& q, const Vec3& omega, const Quaternion& q_cmd) const {
    Quaternion q_err = conjugate(q_cmd) * q;
    if (q_err.w < 0.0) {
      q_err = -q_err;  // shortest-path: keep the scalar part non-negative
    }
    const Vec3 unsaturated = (-kp_) * q_err.vec() - kd_ * omega;
    return clamp_abs(unsaturated, tau_max_);
  }

  double kp() const { return kp_; }
  double kd() const { return kd_; }
  const Vec3& tau_max() const { return tau_max_; }

 private:
  double kp_;
  double kd_;
  Vec3 tau_max_;
};

}  // namespace slew

#endif  // SLEW_CONTROLLER_HPP
