#include "map.hpp"
#include <rog_map_ros/rog_map_ros2.hpp>
#include <stdexcept>

void GridMap::init(rclcpp::Node& node) {
    const auto config = node.declare_parameter<std::string>("map_config_path", "");
    if (config.empty()) throw std::invalid_argument("map_config_path must name the static ROG-Map YAML");
    const auto size = node.declare_parameter<std::vector<double>>("map_size", {30.0, 15.0, 3.0});
    const auto origin = node.declare_parameter<std::vector<double>>("map_origin", {-15.0, -7.5, 0.0});
    if (size.size() != 3 || origin.size() != 3) throw std::invalid_argument("map_size and map_origin require three values");
    size_ = Vector3d(size[0], size[1], size[2]);
    origin_ = Vector3d(origin[0], origin[1], origin[2]);
    map_ = std::make_shared<rog_map::ROGMapROS>(node.shared_from_this(), config);
    const auto cfg = map_->getMapConfig();
    if (!cfg.load_pcd_en || cfg.map_sliding_en || cfg.ros_callback_en)
        throw std::invalid_argument("Table III requires a fixed PCD map with ROS map updates disabled");
    resolution_ = cfg.resolution;
    isize_ = (size_ / resolution_).array().round().cast<int>();
    RCLCPP_INFO(node.get_logger(), "Static ROG-Map: resolution=%.3f, inflation=%d cells, unknown=free, outside=occupied",
                resolution_, cfg.inflation_step);
}

Vector3i GridMap::pos2idx(const Vector3d& pos) const {
    return (pos / resolution_).array().floor().cast<int>();
}

bool GridMap::check_in_map(const Vector3i& idx) const {
    return (idx.array() >= 0).all() && (idx.array() < isize_.array()).all();
}

bool GridMap::check_in_map(const Vector3d& pos) const {
    return (pos.array() >= 0.0).all() && (pos.array() < size_.array()).all();
}

bool GridMap::operator()(const Vector3d& pos) const {
    // ROG-Map itself returns false outside its allocated map. The planning
    // domain is explicit here, independent of its padded storage dimensions.
    if (!check_in_map(pos)) return true;
    const Vector3d world = origin_ + pos;
    return map_->isOccupiedInflate(world);
}

bool GridMap::operator()(const Vector3i& idx) const {
    if (!check_in_map(idx)) return true;
    const Vector3d pos = (idx.cast<double>().array() + 0.5) * resolution_;
    return (*this)(pos);
}

bool GridMap::operator()(int x, int y, int z) const { return (*this)(Vector3i(x, y, z)); }
bool GridMap::operator()(double x, double y, double z) const { return (*this)(Vector3d(x, y, z)); }
