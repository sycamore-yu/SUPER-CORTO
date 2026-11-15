#pragma once

#include <quadrotor_simulator/Quadrotor.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace perfect_drone {

// EGO's rotor dynamics and allocation, with CORTO collective thrust/body rates.
class So3Dynamics {
public:
    explicit So3Dynamics(const Eigen::Vector3d& position, double yaw = 0.0) {
        quad_.setExternalMoment(Eigen::Vector3d::Zero());
        auto state = quad_.getState();
        state.x = position;
        state.R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        state.motor_rpm.setConstant(std::sqrt(
            quad_.getMass() * quad_.getGravity() /
            (4.0 * quad_.getPropellerThrustCoefficient())));
        quad_.setState(state);
    }

    void step(double normalized_thrust, const Eigen::Vector3d& body_rates,
              double dt) {
        const auto& state = quad_.getState();
        const Eigen::Vector3d moment =
            rate_gain_.cwiseProduct(body_rates - state.omega) +
            state.omega.cross(quad_.getInertia() * state.omega);
        // CORTO's simulator defines 0.6 as hover, hence thrust/mass=9.81*u/0.6.
        const double force = quad_.getMass() * 9.81 * normalized_thrust / 0.6;
        const double kf = quad_.getPropellerThrustCoefficient();
        const double km = quad_.getPropellerMomentCoefficient();
        const double d = quad_.getArmLength();
        Eigen::Array4d rpm_squared;
        rpm_squared <<
            force / (4 * kf) - moment.y() / (2 * d * kf) + moment.z() / (4 * km),
            force / (4 * kf) + moment.y() / (2 * d * kf) + moment.z() / (4 * km),
            force / (4 * kf) + moment.x() / (2 * d * kf) - moment.z() / (4 * km),
            force / (4 * kf) - moment.x() / (2 * d * kf) - moment.z() / (4 * km);
        const auto rpm = rpm_squared.max(0.0).sqrt().eval();
        quad_.setInput(rpm[0], rpm[1], rpm[2], rpm[3]);
        quad_.step(dt);
    }

    const QuadrotorSimulator::Quadrotor::State& state() const {
        return quad_.getState();
    }

private:
    QuadrotorSimulator::Quadrotor quad_;
    // EGO so3_control/config/gains_hummingbird.yaml, gains/ang.
    const Eigen::Vector3d rate_gain_{0.07, 0.07, 0.02};
};

}  // namespace perfect_drone

