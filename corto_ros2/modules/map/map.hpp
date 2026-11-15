#pragma once

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace rog_map { class ROGMapROS; }

using Eigen::Vector3d;
using Eigen::Vector3i;

// CORTO queries local coordinates; ROG-Map stores world coordinates.
class GridMap {
public:
    void init(rclcpp::Node& node);
    Vector3i pos2idx(const Vector3d& pos) const;
    bool check_in_map(const Vector3i& idx) const;
    bool check_in_map(const Vector3d& pos) const;
    bool operator()(const Vector3i& idx) const;
    bool operator()(const Vector3d& pos) const;
    bool operator()(int x, int y, int z) const;
    bool operator()(double x, double y, double z) const;
    bool isLineFree(const Vector3d& start_world, const Vector3d& end_world) const;
    bool nearbyFreeEndpoint(const Vector3d& preferred_world, Vector3d& free_world) const;
    double resolution() const { return resolution_; }
    Vector3d size() const { return size_; }
    Vector3i isize() const { return isize_; }
    Vector3d origin() const { return origin_; }

private:
    std::shared_ptr<rog_map::ROGMapROS> map_;
    double resolution_ = 0.1;
    Vector3d size_{30.0, 15.0, 3.0};
    Vector3d origin_{-15.0, -7.5, 0.0};
    Vector3i isize_{300, 150, 30};
};
