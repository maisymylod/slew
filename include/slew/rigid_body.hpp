// Rigid-body attitude dynamics: Euler's rotational equations plus quaternion
// kinematics, advanced with a fixed-step RK4 integrator.
//
//   q_dot = 0.5 * q (x) [0, omega]                 (kinematics, body rates)
//   omega_dot = J^-1 ( tau - omega x (J omega) )   (Euler's equation)
//
// J is diagonal (principal axes). Torque is held constant across the step
// (zero-order hold from the controller). Header-only, no allocation in step().
#ifndef SLEW_RIGID_BODY_HPP
#define SLEW_RIGID_BODY_HPP

#include "slew/quaternion.hpp"
#include "slew/vec3.hpp"

namespace slew {

struct AttitudeState {
  Quaternion q;       // body-to-inertial attitude
  Vec3 omega;         // body angular velocity (rad/s)
};

inline AttitudeState operator+(const AttitudeState& a, const AttitudeState& b) {
  return {a.q + b.q, a.omega + b.omega};
}
inline AttitudeState operator*(double s, const AttitudeState& a) {
  return {s * a.q, s * a.omega};
}

class RigidBody {
 public:
  // Principal moments of inertia (kg m^2), strictly positive.
  explicit RigidBody(const Vec3& inertia) : J_(inertia) {}

  const Vec3& inertia() const { return J_; }

  Vec3 angular_momentum(const AttitudeState& s) const {
    return {J_.x * s.omega.x, J_.y * s.omega.y, J_.z * s.omega.z};
  }
  // Rotational kinetic energy, 0.5 * omega . (J omega).
  double kinetic_energy(const AttitudeState& s) const {
    return 0.5 * dot(s.omega, angular_momentum(s));
  }

  // Continuous-time derivative of the state for a constant body torque.
  AttitudeState derivative(const AttitudeState& s, const Vec3& tau) const {
    const Quaternion omega_q{0.0, s.omega.x, s.omega.y, s.omega.z};
    const Quaternion q_dot = 0.5 * (s.q * omega_q);

    const Vec3 Jw = {J_.x * s.omega.x, J_.y * s.omega.y, J_.z * s.omega.z};
    const Vec3 gyroscopic = cross(s.omega, Jw);
    const Vec3 net = tau - gyroscopic;
    const Vec3 omega_dot = {net.x / J_.x, net.y / J_.y, net.z / J_.z};

    return {q_dot, omega_dot};
  }

  // One fixed-step RK4 update with zero-order-hold torque, then renormalize
  // the attitude to stay on the unit sphere.
  AttitudeState step(const AttitudeState& s, const Vec3& tau, double dt) const {
    const AttitudeState k1 = derivative(s, tau);
    const AttitudeState k2 = derivative(s + (0.5 * dt) * k1, tau);
    const AttitudeState k3 = derivative(s + (0.5 * dt) * k2, tau);
    const AttitudeState k4 = derivative(s + dt * k3, tau);

    AttitudeState next = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    next.q = normalized(next.q);
    return next;
  }

 private:
  Vec3 J_;
};

}  // namespace slew

#endif  // SLEW_RIGID_BODY_HPP
