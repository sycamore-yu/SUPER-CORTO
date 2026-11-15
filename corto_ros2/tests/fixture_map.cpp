#include "map/map.hpp"
#include <rog_map/rog_map_core/raycaster.h>
// Deterministic occupancy input for Topo-PRM regression. The production ROG
// adapter is exercised separately by the ROS integration experiment.
void GridMap::init(rclcpp::Node&) {}
Vector3i GridMap::pos2idx(const Vector3d& p) const { return (p / resolution_).array().floor().cast<int>(); }
bool GridMap::check_in_map(const Vector3i& i) const { return (i.array() >= 0).all() && (i.array() < isize_.array()).all(); }
bool GridMap::check_in_map(const Vector3d& p) const { return (p.array() >= 0).all() && (p.array() < size_.array()).all(); }
bool GridMap::operator()(const Vector3i& i) const {
    return !check_in_map(i) || (i.x() >= 145 && i.x() < 155 && i.y() >= 70 && i.y() < 80);
}
bool GridMap::operator()(const Vector3d& p) const { return (*this)(pos2idx(p)); }
bool GridMap::operator()(int x, int y, int z) const { return (*this)(Vector3i(x,y,z)); }
bool GridMap::operator()(double x, double y, double z) const { return (*this)(Vector3d(x,y,z)); }
bool GridMap::isLineFree(const Vector3d& a, const Vector3d& b) const {
    if ((*this)(Vector3d(a - origin_)) || (*this)(Vector3d(b - origin_))) return false;
    rog_map::raycaster::RayCaster ray;
    ray.setResolution(resolution_);
    ray.setInput(a, b);
    Vector3d p;
    while (ray.step(p)) if ((*this)(Vector3d(p - origin_))) return false;
    return true;
}
