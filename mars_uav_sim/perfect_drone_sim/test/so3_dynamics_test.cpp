#include <perfect_drone_sim/so3_dynamics.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    constexpr double dt = 0.001;
    const Eigen::Vector3d initial(0.0, 0.0, 1.5);
    perfect_drone::So3Dynamics hover(initial);
    for (int i = 0; i < 3000; ++i) hover.step(0.6, Eigen::Vector3d::Zero(), dt);
    const double hover_drift = (hover.state().x - initial).norm();
    require(hover_drift < 1e-7, "CORTO u=0.6 must hover");

    QuadrotorSimulator::Quadrotor motor;
    motor.setExternalMoment(Eigen::Vector3d::Zero());
    motor.setInput(15000.0, 15000.0, 15000.0, 15000.0);
    for (int i = 0; i < 100; ++i) motor.step(dt);
    const double expected_rpm = 15000.0 * (1.0 - std::exp(-0.1 / motor.getMotorTimeConstant()));
    const double motor_error = std::abs(motor.getState().motor_rpm[0] - expected_rpm);
    require(motor_error < 1e-4, "EGO first-order motor response changed");

    perfect_drone::So3Dynamics rates(Eigen::Vector3d(0.0, 0.0, 10.0));
    const Eigen::Vector3d desired(0.15, -0.1, 0.2);
    for (int i = 0; i < 3000; ++i) rates.step(0.6, desired, dt);
    const double rate_error = (rates.state().omega - desired).norm();
    require(rate_error < 1e-4, "body-rate feedback or rotor allocation sign is wrong");
    require(std::abs(rates.state().R.determinant() - 1.0) < 1e-8,
            "SO(3) rotation is not normalized");

    perfect_drone::So3Dynamics thrust(initial);
    for (int i = 0; i < 100; ++i) thrust.step(0.7, Eigen::Vector3d::Zero(), dt);
    require(thrust.state().v.z() > 0.05, "positive collective increment must accelerate upward");
    std::cout << "{\"hover_drift_m\":" << hover_drift
              << ",\"motor_response_error_rpm\":" << motor_error
              << ",\"body_rate_error_rad_s\":" << rate_error
              << ",\"collective_step_vz_m_s\":" << thrust.state().v.z()
              << "}" << std::endl;
}

