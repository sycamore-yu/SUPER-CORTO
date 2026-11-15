#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <atomic>
#include <iomanip>
#include <sstream>
#include "path_searching/topo_prm.hpp"
#include "corridor/corGen.hpp"

class RosInterface {
    using Marker = visualization_msgs::msg::Marker;
    rclcpp::Node& node_;
    rclcpp::Publisher<Marker>::SharedPtr predicted_, paths_, graph_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridors_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr control_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_, status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_;
    const std::chrono::steady_clock::time_point start_wall_ = std::chrono::steady_clock::now();
    std::atomic<double> sim_time_{0.0};

    Marker marker(const std::string& ns, int id, int type) {
        Marker m;
        m.header.frame_id = "world";
        m.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(sim_time_.load() * 1e9)));
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.025;
        m.color.a = 1.0;
        return m;
    }
    static geometry_msgs::msg::Point point(const Eigen::Vector3d& v) {
        geometry_msgs::msg::Point p;
        p.x = v.x(); p.y = v.y(); p.z = v.z();
        return p;
    }
public:
    explicit RosInterface(rclcpp::Node& node) : node_(node) {
        predicted_ = node.create_publisher<Marker>("/corto/predict_traj", 10);
        paths_ = node.create_publisher<Marker>("/corto/filter_path", 10);
        graph_ = node.create_publisher<Marker>("/corto/graph_nodes", 10);
        corridors_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/corto/corridor_markers", 10);
        control_ = node.create_publisher<geometry_msgs::msg::WrenchStamped>("/corto/control", 100);
        state_ = node.create_publisher<nav_msgs::msg::Odometry>("/corto/odom", rclcpp::QoS(100).transient_local());
        clock_ = node.create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).transient_local());
        events_ = node.create_publisher<std_msgs::msg::String>("/corto/events", 1000);
        status_ = node.create_publisher<std_msgs::msg::String>("/corto/status", rclcpp::QoS(1).transient_local());
        stop_ = node.create_publisher<std_msgs::msg::Bool>("/corto/stop", 10);
    }
    void set_sim_time(double t) { sim_time_.store(t); }
    void event(const std::string& kind, const std::string& fields = "") {
        std::ostringstream out;
        out << std::setprecision(17) << "{\"event\":\"" << kind << "\",\"sim_time_s\":" << sim_time_.load()
            << ",\"wall_time_s\":" << std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall_).count();
        if (!fields.empty()) out << "," << fields;
        out << "}";
        std_msgs::msg::String msg;
        msg.data = out.str();
        events_->publish(msg);
    }
    void status(const std::string& value) {
        std_msgs::msg::String msg; msg.data = value; status_->publish(msg);
    }
    void publish_stop() {
        std_msgs::msg::Bool msg; msg.data = true; stop_->publish(msg);
    }
    pcl::PointCloud<pcl::PointXYZ> grid_to_cloud(GridMap& map) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        const double r = map.resolution();
        const Eigen::Vector3d size = map.size(), origin = map.origin();
        for (double z = r / 2.0; z < size.z(); z += r) {
            for (double y = r / 2.0; y < size.y(); y += r)
                for (double x = r / 2.0; x < size.x(); x += r)
                    if (map(Eigen::Vector3d(x, y, z)))
                        cloud.points.emplace_back(x + origin.x(), y + origin.y(), z + origin.z());
        }
        cloud.width = cloud.points.size(); cloud.height = 1; cloud.is_dense = true;
        return cloud;
    }
    void publish_control(const Eigen::Vector4d& u) {
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.frame_id = "body";
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(sim_time_.load() * 1e9)));
        msg.wrench.force.x = u(0); msg.wrench.force.y = u(1); msg.wrench.force.z = u(2);
        msg.wrench.torque.x = u(3);
        control_->publish(msg);
    }
    void publish_state(const Eigen::Matrix<double, 10, 1>& x, const Eigen::Vector3d& body_rates, double elapsed) {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "world"; odom.child_frame_id = "body";
        odom.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(elapsed * 1e9)));
        odom.pose.pose.position = point(x.head<3>());
        odom.pose.pose.orientation.w = x(6); odom.pose.pose.orientation.x = x(7);
        odom.pose.pose.orientation.y = x(8); odom.pose.pose.orientation.z = x(9);
        odom.twist.twist.linear.x = x(3); odom.twist.twist.linear.y = x(4); odom.twist.twist.linear.z = x(5);
        odom.twist.twist.angular.x = body_rates.x();
        odom.twist.twist.angular.y = body_rates.y();
        odom.twist.twist.angular.z = body_rates.z();
        rosgraph_msgs::msg::Clock clock;
        clock.clock = odom.header.stamp;
        clock_->publish(clock);
        state_->publish(odom);
    }
    void publish_predict_traj(const std::vector<Eigen::Vector3d>& points) {
        auto m = marker("prediction", 0, Marker::LINE_STRIP);
        m.color.r = 1.0; m.color.g = 0.5;
        for (const auto& p : points) m.points.push_back(point(p));
        predicted_->publish(m);
    }
    void drawFilterPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, double width, int id, double alpha) {
        auto clear = marker("paths", 0, Marker::LINE_STRIP); clear.action = Marker::DELETEALL; paths_->publish(clear);
        for (size_t i = 0; i < paths.size(); ++i) {
            auto m = marker("paths", id + i, Marker::LINE_STRIP);
            m.color.b = 1.0; m.color.g = 0.5; m.color.a = alpha; m.scale.x = width;
            for (const auto& p : paths[i]) m.points.push_back(point(p));
            paths_->publish(m);
        }
    }
    void visualizeGraphNodes(const std::list<GraphNode::Ptr>& nodes) {
        auto m = marker("topology", 0, Marker::SPHERE_LIST);
        m.scale.x = m.scale.y = m.scale.z = 0.12; m.color.g = 1.0;
        for (const auto& n : nodes) m.points.push_back(point(n->pos_));
        graph_->publish(m);
    }
    void visualizeCorridorMarkers(const CorridorVisualization& corridor) {
        visualization_msgs::msg::MarkerArray arr;
        auto clear = marker("corridor", 0, Marker::LINE_STRIP); clear.action = Marker::DELETEALL;
        arr.markers.push_back(clear);
        for (size_t i = 0; i < corridor.ellipse_pts_world.size(); i += 2) {
            auto m = marker("corridor", i + 1, Marker::LINE_STRIP);
            m.color.r = 0.7; m.color.a = 0.6; m.scale.x = 0.01;
            for (const auto& p : corridor.ellipse_pts_world[i]) m.points.push_back(point(p));
            if (!m.points.empty()) m.points.push_back(m.points.front());
            arr.markers.push_back(std::move(m));
        }
        corridors_->publish(arr);
    }
};
