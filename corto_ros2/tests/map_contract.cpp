#include "map/map.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto dir = std::filesystem::temp_directory_path() /
                     ("super_corto_map_contract_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const auto pcd = dir / "one_voxel.pcd";
    std::ofstream(pcd) << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0.05 0.05 1.55 1\n";
    const auto config = dir / "map.yaml";
    std::ofstream(config) << "rog_map:\n  resolution: 0.1\n  inflation_resolution: 0.1\n"
        "  inflation_step: 1\n  map_size: [30, 15, 3]\n  fix_map_origin: [0, 0, 1.5]\n"
        "  load_pcd_en: true\n  pcd_name: " << pcd.string() << "\n"
        "  virtual_ground_height: -10\n  virtual_ceil_height: 10\n"
        "  map_sliding: {enable: false}\n  ros_callback: {enable: false}\n"
        "  visualization: {enable: false, range: [30, 15, 3]}\n"
        "  point_filt_num: 1\n  intensity_thresh: -10\n"
        "  raycasting: {enable: false, p_hit: 0.9, p_occ: 0.85, p_max: 0.98, p_miss: 0.49, p_free: 0.499}\n";
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("map_config_path", config.string())});
    auto node = std::make_shared<rclcpp::Node>("map_contract", options);
    GridMap map;
    map.init(*node);
    int failures = 0;
    auto check = [&](bool condition, const char* name) {
        std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
        if (!condition) ++failures;
    };
    check((map.origin() - Vector3d(-15, -7.5, 0)).norm() == 0, "world-to-local origin");
    check(map(Vector3d(15.05, 7.55, 1.55)), "PCD occupied world voxel");
    check(map(Vector3d(15.15, 7.55, 1.55)), "ROG one-cell inflation");
    check(!map(Vector3d(15.35, 7.55, 1.55)), "outside inflation remains unknown/free");
    check(!map(Vector3d(1, 1, 1)), "static unknown is free");
    check(map(Vector3d(-1e-9, 1, 1)), "negative local coordinate is outside");
    check(map(Vector3d(30, 1, 1)), "upper x boundary is outside");
    check(map(Vector3d(1, 15, 1)), "upper y boundary is outside");
    check(map(Vector3d(1, 1, 3)), "upper z boundary is outside");
    check(!map(Vector3d(0.05, 0.05, 0.05)), "no implicit ground inflation inside scene");
    check(map.pos2idx(Vector3d(-0.001, 0, 0)).x() == -1, "floor indexing for negative positions");
    rclcpp::shutdown();
    return failures ? 1 : 0;
}
