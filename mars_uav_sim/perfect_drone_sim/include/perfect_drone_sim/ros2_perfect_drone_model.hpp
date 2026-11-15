#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <marsim_render/marsim_render.hpp>
#include <perfect_drone_sim/config.hpp>

namespace perfect_drone {

// CORTO owns the state and clock. This node only publishes and renders that state.
class PerfectDrone : public rclcpp::Node {
    Config cfg_;
    std::shared_ptr<marsim::MarsimRender> renderer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_, global_pc_pub_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    nav_msgs::msg::Odometry state_;
    nav_msgs::msg::Path path_;
    bool have_state_ = false;

public:
    PerfectDrone() : Node("perfect_tracking") {
        const auto name = declare_parameter<std::string>("config_name", "dense.yaml");
        const auto config = name.front() == '/' ? name : std::string(ROOT_DIR) + "config/" + name;
        cfg_ = Config(config);
        renderer_ = std::make_shared<marsim::MarsimRender>(config);
        transform_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_slam/odom", 100);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/lidar_slam/pose", 100);
        robot_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
        local_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", rclcpp::SensorDataQoS());
        global_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/global_pc", rclcpp::QoS(1).transient_local());
        state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/corto/odom", rclcpp::QoS(100).transient_local(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                state_ = *msg;
                have_state_ = true;
                publishState();
            });
        lidar_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / cfg_.sensing_rate),
                                        [this]() { publishCloud(); });
        pcl::PointCloud<marsim::PointType>::Ptr cloud(new pcl::PointCloud<marsim::PointType>);
        renderer_->getGlobalMap(cloud);
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*cloud, message);
        message.header.frame_id = "world";
        global_pc_pub_->publish(message);
    }

private:
    void publishCloud() {
        if (!have_state_) return;
        const auto& p = state_.pose.pose.position;
        const auto& q = state_.pose.pose.orientation;
        pcl::PointCloud<marsim::PointType>::Ptr cloud(new pcl::PointCloud<marsim::PointType>);
        renderer_->renderOnceInWorld(Eigen::Vector3f(p.x, p.y, p.z),
                                     Eigen::Quaternionf(q.w, q.x, q.y, q.z),
                                     rclcpp::Time(state_.header.stamp).seconds(), cloud);
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*cloud, message);
        message.header = state_.header;
        local_pc_pub_->publish(message);
    }

    void publishState() {
        odom_pub_->publish(state_);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = state_.header;
        pose.pose = state_.pose.pose;
        pose_pub_->publish(pose);

        geometry_msgs::msg::TransformStamped transform;
        transform.header = state_.header;
        transform.child_frame_id = "perfect_drone";
        transform.transform.translation.x = pose.pose.position.x;
        transform.transform.translation.y = pose.pose.position.y;
        transform.transform.translation.z = pose.pose.position.z;
        transform.transform.rotation = pose.pose.orientation;
        transform_pub_->sendTransform(transform);

        visualization_msgs::msg::Marker mesh;
        mesh.header = state_.header;
        mesh.ns = "mesh";
        mesh.id = 0;
        mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh.action = visualization_msgs::msg::Marker::ADD;
        mesh.pose = pose.pose;
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;
        mesh.mesh_resource = cfg_.mesh_resource;
        mesh.mesh_use_embedded_materials = true;
        mesh.color.a = 1.0;
        robot_pub_->publish(mesh);

        path_.header = state_.header;
        path_.poses.push_back(pose);
        if (path_.poses.size() % 10 == 1) path_pub_->publish(path_);
    }
};
}  // namespace perfect_drone
