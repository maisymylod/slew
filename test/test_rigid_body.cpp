// Physics checks on the plant: conservation laws a correct rigid-body
// integrator must respect, plus basic integrator sanity.
#include <cmath>

#include "framework.hpp"
#include "slew/rigid_body.hpp"

using namespace slew;

SLEW_TEST(torque_free_conserves_momentum_and_energy) {
  // Asymmetric inertia, tumbling, no applied torque. |H| and rotational
  // kinetic energy are invariants; a good integrator holds them across a long
  // run. (The free asymmetric body's body-frame omega components move, so this
  // also exercises the gyroscopic coupling term.)
  const RigidBody body({10.0, 12.0, 8.0});
  AttitudeState s{Quaternion{}, Vec3{0.3, -0.2, 0.5}};

  const double H0 = norm(body.angular_momentum(s));
  const double E0 = body.kinetic_energy(s);

  const double dt = 1.0e-3;
  for (int i = 0; i < 20000; ++i) {  // 20 s
    s = body.step(s, Vec3{}, dt);
  }

  CHECK_NEAR(norm(body.angular_momentum(s)), H0, 1e-6 * H0);
  CHECK_NEAR(body.kinetic_energy(s), E0, 1e-6 * E0);
  CHECK_NEAR(norm(s.q), 1.0, 1e-9);  // attitude stayed on the unit sphere
}

SLEW_TEST(symmetric_body_holds_angular_velocity) {
  // With equal moments the gyroscopic term vanishes, so a torque-free spin is
  // constant in both magnitude and direction.
  const RigidBody body({9.0, 9.0, 9.0});
  AttitudeState s{Quaternion{}, Vec3{0.1, 0.0, 0.2}};
  const Vec3 w0 = s.omega;

  const double dt = 1.0e-3;
  for (int i = 0; i < 5000; ++i) {
    s = body.step(s, Vec3{}, dt);
  }
  CHECK_NEAR(s.omega.x, w0.x, 1e-9);
  CHECK_NEAR(s.omega.y, w0.y, 1e-9);
  CHECK_NEAR(s.omega.z, w0.z, 1e-9);
}

SLEW_TEST(constant_torque_gives_expected_spin_up) {
  // From rest, a constant torque about a principal axis spins the body up at
  // omega_dot = tau / J. After time T the rate is tau*T/J (single-axis, no
  // coupling), which the integrator should match closely.
  const RigidBody body({10.0, 12.0, 8.0});
  AttitudeState s{Quaternion{}, Vec3{}};
  const Vec3 tau{0.0, 0.0, 0.5};

  const double dt = 1.0e-3;
  const int n = 1000;  // 1 s
  for (int i = 0; i < n; ++i) {
    s = body.step(s, tau, dt);
  }
  const double expected = tau.z * (n * dt) / body.inertia().z;
  CHECK_NEAR(s.omega.z, expected, 1e-6);
  CHECK_NEAR(s.omega.x, 0.0, 1e-12);
  CHECK_NEAR(s.omega.y, 0.0, 1e-12);
}
