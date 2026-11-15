#include "perfect_drone_sim/ros2_perfect_drone_model.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<perfect_drone::PerfectDrone>();
    try {
        rclcpp::spin(node);
    } catch (const std::runtime_error&) {
        // A callback may observe shutdown between its ROS API calls.
        if (rclcpp::ok()) throw;
    }
    node.reset();
    rclcpp::shutdown();
    return 0;
}
