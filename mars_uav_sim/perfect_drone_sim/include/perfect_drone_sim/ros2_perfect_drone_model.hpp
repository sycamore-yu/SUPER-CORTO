#ifndef _PERFECT_DRONE_SIM_HPP_
#define _PERFECT_DRONE_SIM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mars_quadrotor_msgs/msg/position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "string"
#include "Eigen/Dense"
#include "nav_msgs/msg/path.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <marsim_render/marsim_render.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "perfect_drone_sim/config.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <perfect_drone_sim/so3_dynamics.hpp>
#include <atomic>
#include <mutex>
#include <stdexcept>
#include <thread>


typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> TimePosPair;

namespace perfect_drone {
    class PerfectDrone : public rclcpp::Node {
        std::shared_ptr<tf2_ros::TransformBroadcaster> br_map_ego_;

        Config cfg_;
        std::shared_ptr<marsim::MarsimRender> render_ptr_;
        double sys_start_t;
        rclcpp::TimerBase::SharedPtr global_pc_pub_timer_;
        rclcpp::TimerBase::SharedPtr local_pc_pub_timer_;
        rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ctbr_sub_;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
        std::unique_ptr<So3Dynamics> dynamics_;
        std::mutex dynamics_mutex_;
        Eigen::Vector3d body_rate_command_{Eigen::Vector3d::Zero()};
        double thrust_command_{0.6};
        std::atomic<int64_t> simulation_ns_{0};
        std::atomic<bool> physics_running_{false};
        std::thread physics_thread_;
        rclcpp::executors::SingleThreadedExecutor control_executor_;
        rclcpp::CallbackGroup::SharedPtr global_pc_pub_cbk_group, local_pc_pub_cbk_group,
                cmd_sub_cbk_group;


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc_pub_;
        Vec3 position_, velocity_;
        double yaw_;
        Eigen::Quaterniond q_;
        std::string mesh_resource_;

        nav_msgs::msg::Odometry odom_;
        nav_msgs::msg::Path path_;
        struct StateSnapshot {
            Vec3 position, velocity, angular_velocity;
            Eigen::Quaterniond orientation;
            Eigen::Array4d motor_rpm;
            int64_t stamp_ns;
        };


    public:
        PerfectDrone(): Node("perfect_tracking") {
            // TODO: The current implementation uses a lenient QoS configuration for message transmission.
            const rclcpp::QoS qos(rclcpp::QoS(100)
                                          .best_effort()
                                          .keep_last(100)
                                          .durability_volatile());

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))
            std::string dft_cfg_path = "dense.yaml";
            std::string cfg_path, cfg_name;
            this->declare_parameter<std::string>("config_name", dft_cfg_path);

            if(this->get_parameter("config_name", cfg_name)){
                cfg_path = (!cfg_name.empty() && cfg_name.front() == '/') ?
                           cfg_name : CONFIG_FILE_DIR(cfg_name);
                RCLCPP_WARN(this->get_logger(), " -- [MissionPlanner] Load config by file name: %s", cfg_path.c_str());
            }

            cfg_ = Config(cfg_path);
            cfg_.dynamics_mode = this->declare_parameter<std::string>(
                    "dynamics_mode", cfg_.dynamics_mode);
            if (cfg_.dynamics_mode != "perfect" && cfg_.dynamics_mode != "so3") {
                throw std::invalid_argument("dynamics_mode must be perfect or so3");
            }

            // 创建 TransformBroadcaster

            br_map_ego_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


            // 初始化 render_ptr_
            render_ptr_ = std::make_shared<marsim::MarsimRender>(cfg_path);

            // 订阅命令
            rclcpp::SubscriptionOptions so;
            cmd_sub_cbk_group = this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive, false);
            so.callback_group = cmd_sub_cbk_group;
            cmd_sub_ = this->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
                    "/planning/pos_cmd", qos, std::bind(&PerfectDrone::cmdCallback, this, std::placeholders::_1),
                    so
            );
            ctbr_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                    "/corto/control", rclcpp::QoS(1).reliable(),
                    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                        std::lock_guard<std::mutex> lock(dynamics_mutex_);
                        body_rate_command_ << msg->wrench.force.x, msg->wrench.force.y,
                                              msg->wrench.force.z;
                        thrust_command_ = msg->wrench.torque.x;
                    }, so);
            control_executor_.add_callback_group(cmd_sub_cbk_group, this->get_node_base_interface());
            clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
                    "/clock", rclcpp::QoS(1).best_effort());
            motor_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/simulator/motor_rpm", qos);

            // 发布 Odometry 消息
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_slam/odom", qos);

            // 发布 Pose 消息
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lidar_slam/pose", qos);

            // 发布 Robot Marker
            robot_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot", qos);

            // 发布 Path
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);

            // 发布 PointCloud2 消息
            local_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos);

            global_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pc", qos);


            position_ = cfg_.init_pos;
            velocity_.setZero();
            yaw_ = cfg_.init_yaw;
            mesh_resource_ = cfg_.mesh_resource;
            q_ = Eigen::AngleAxisd(yaw_, Vec3::UnitZ());
            if (cfg_.dynamics_mode == "so3") {
                dynamics_ = std::make_unique<So3Dynamics>(position_, yaw_);
            }
            odom_.header.frame_id = "world";
            path_.poses.clear();
            path_.header.frame_id = "world";
            path_.header.stamp = this->get_clock()->now();


            global_pc_pub_cbk_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            global_pc_pub_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(1),
                    std::bind(&PerfectDrone::publishGlobalPC, this),
                    global_pc_pub_cbk_group
            );

            const int publish_dt_ms = static_cast<int>((1.0 / cfg_.sensing_rate) * 1000);
            local_pc_pub_cbk_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            local_pc_pub_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(publish_dt_ms),
                    std::bind(&PerfectDrone::publishPC, this),
                    local_pc_pub_cbk_group
            );

            sys_start_t = 0.0;
            physics_running_ = true;
            physics_thread_ = std::thread(&PerfectDrone::runPhysics, this);
        }

        double getSensingRate() {
            return cfg_.sensing_rate;
        }


        void publishPC() {
            const auto state = currentState();
            pcl::PointCloud<marsim::PointType>::Ptr local_map(new pcl::PointCloud<marsim::PointType>);
            const double cur_t = state.stamp_ns * 1e-9;
            render_ptr_->renderOnceInWorld(state.position.cast<float>(),
                                          state.orientation.cast<float>(), cur_t, local_map);
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*local_map, pc_msg);
            pc_msg.header.frame_id = "world";
            pc_msg.header.stamp = rclcpp::Time(state.stamp_ns);
            local_pc_pub_->publish(pc_msg);
        }

        ~PerfectDrone() {
            physics_running_ = false;
            if (physics_thread_.joinable()) physics_thread_.join();
        }

    private:
        void runPhysics() {
            try {
            constexpr int64_t step_ns = 1000000;
            auto next_step = std::chrono::steady_clock::now();
            int clock_decimation = 0;
            while (physics_running_ && rclcpp::ok()) {
                next_step += std::chrono::nanoseconds(step_ns);
                control_executor_.spin_some();
                {
                    std::lock_guard<std::mutex> lock(dynamics_mutex_);
                    if (dynamics_) {
                        dynamics_->step(thrust_command_, body_rate_command_, 0.001);
                    }
                    simulation_ns_ += step_ns;
                }
                if (++clock_decimation == 10) {
                    rosgraph_msgs::msg::Clock msg;
                    msg.clock = rclcpp::Time(simulation_ns_.load());
                    clock_pub_->publish(msg);
                    publishOdom();
                    clock_decimation = 0;
                }
                std::this_thread::sleep_until(next_step);
            }
            } catch (const std::runtime_error&) {
                // SIGINT can invalidate the ROS context during a publish/spin call.
                if (rclcpp::ok()) throw;
            }
        }

        StateSnapshot currentState() {
            std::lock_guard<std::mutex> lock(dynamics_mutex_);
            if (dynamics_) {
                const auto& state = dynamics_->state();
                return {state.x, state.v, state.omega, Eigen::Quaterniond(state.R),
                        state.motor_rpm, simulation_ns_.load()};
            }
            return {position_, velocity_, Vec3::Zero(), q_, Eigen::Array4d::Zero(),
                    simulation_ns_.load()};
        }

        void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
            if (cfg_.dynamics_mode != "perfect") return;
            Vec3 pos(msg->position.x, msg->position.y, msg->position.z);
            Vec3 vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
            Vec3 acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
            double yaw = msg->yaw;
            updateFlatness(pos, vel, acc, yaw);
        }


        void publishGlobalPC() {
            static int last_sub_num = 0;
            // update sub num
            int sub_num = this->count_subscribers("/global_pc");
            double cur_t = simulation_ns_.load() * 1e-9 - sys_start_t;
            if (sub_num > 0 && last_sub_num != sub_num || (cur_t > 5.0 && cur_t < 5.1)) {
                pcl::PointCloud<marsim::PointType>::Ptr global_map(new pcl::PointCloud<marsim::PointType>);
                render_ptr_->getGlobalMap(global_map);
                sensor_msgs::msg::PointCloud2 pc_msg;
                pcl::toROSMsg(*global_map, pc_msg);
                pc_msg.header.frame_id = "world";
                pc_msg.header.stamp = rclcpp::Time(simulation_ns_.load());
                global_pc_pub_->publish(pc_msg);
                std::cout << "Publish global map size: " << global_map->size() << std::endl;
            }
            last_sub_num = sub_num;
        }

        void publishOdom() {
            const auto state = currentState();
            odom_.pose.pose.position.x = state.position.x();
            odom_.pose.pose.position.y = state.position.y();
            odom_.pose.pose.position.z = state.position.z();

            odom_.pose.pose.orientation.x = state.orientation.x();
            odom_.pose.pose.orientation.y = state.orientation.y();
            odom_.pose.pose.orientation.z = state.orientation.z();
            odom_.pose.pose.orientation.w = state.orientation.w();

            odom_.twist.twist.linear.x = state.velocity.x();
            odom_.twist.twist.linear.y = state.velocity.y();
            odom_.twist.twist.linear.z = state.velocity.z();
            odom_.twist.twist.angular.x = state.angular_velocity.x();
            odom_.twist.twist.angular.y = state.angular_velocity.y();
            odom_.twist.twist.angular.z = state.angular_velocity.z();

            odom_.header.stamp = rclcpp::Time(state.stamp_ns);


            odom_pub_->publish(odom_);
            if (dynamics_) {
                std_msgs::msg::Float64MultiArray msg;
                msg.data.assign(state.motor_rpm.data(), state.motor_rpm.data() + 4);
                motor_pub_->publish(msg);
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.pose = odom_.pose.pose;
            pose.header = odom_.header;
            pose_pub_->publish(pose);

            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = odom_.header.stamp;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "perfect_drone";
            transformStamped.transform.translation.x = odom_.pose.pose.position.x;
            transformStamped.transform.translation.y = odom_.pose.pose.position.y;
            transformStamped.transform.translation.z = odom_.pose.pose.position.z;
            transformStamped.transform.rotation.x = odom_.pose.pose.orientation.x;
            transformStamped.transform.rotation.y = odom_.pose.pose.orientation.y;
            transformStamped.transform.rotation.z = odom_.pose.pose.orientation.z;
            transformStamped.transform.rotation.w = odom_.pose.pose.orientation.w;

            // 发布变换
            br_map_ego_->sendTransform(transformStamped);

            visualization_msgs::msg::Marker meshROS;
            meshROS.header.frame_id = "world";
            meshROS.header.stamp = odom_.header.stamp;
            meshROS.ns = "mesh";
            meshROS.id = 0;
            meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            meshROS.action = visualization_msgs::msg::Marker::ADD;
            meshROS.pose.position = odom_.pose.pose.position;
            meshROS.pose.orientation = odom_.pose.pose.orientation;
            meshROS.scale.x = 1;
            meshROS.scale.y = 1;
            meshROS.scale.z = 1;
            meshROS.mesh_resource = mesh_resource_;
            meshROS.mesh_use_embedded_materials = true;
            meshROS.color.a = 1.0;
            meshROS.color.r = 0.0;
            meshROS.color.g = 0.0;
            meshROS.color.b = 0.0;
            robot_pub_->publish(meshROS);
            static int slow_down = 0;
            if (slow_down++ % 10 == 0) {
                if ((state.position.head(2) - Vec3(0, -50, 1.5).head(2)).norm() < 1) {
                    path_.poses.clear();
                    path_.poses.reserve(10000);
                }
                path_.poses.push_back(pose);
                path_.header = odom_.header;
                path_pub_->publish(path_);
            }
        }

        void updateFlatness(const Vec3& pos, const Vec3& vel,
                            const Vec3& acc, const double yaw) {
            std::lock_guard<std::mutex> lock(dynamics_mutex_);
            Vec3 gravity_ = 9.80 * Eigen::Vector3d(0, 0, 1);
            position_ = pos;
            velocity_ = vel;
            double a_T = (gravity_ + acc).norm();
            Eigen::Vector3d xB, yB, zB;
            Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);

            zB = (gravity_ + acc).normalized();
            yB = ((zB).cross(xC)).normalized();
            xB = yB.cross(zB);
            Eigen::Matrix3d R;
            R << xB, yB, zB;
            q_ = Eigen::Quaterniond(R);
        }
    };
}


#endif
