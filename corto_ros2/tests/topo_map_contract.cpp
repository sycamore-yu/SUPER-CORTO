#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <random>
#include <memory>
#include <chrono>
#include <unistd.h>
#include "map/map.hpp"
#include <rog_map/rog_map_core/raycaster.h>
#include "path_searching/topo_prm.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto dir = std::filesystem::temp_directory_path() /
                     ("corto_topo_map_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const bool real_scene = argc > 1;
    const auto pcd = real_scene ? std::filesystem::path(argv[1]) : dir / "voxel.pcd";
    if (!real_scene) std::ofstream(pcd) << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0.05 0.05 1.55 1\n";
    const auto config = dir / "map.yaml";
    std::ofstream(config) << "rog_map:\n  resolution: 0.1\n  inflation_resolution: 0.1\n"
        "  inflation_step: " << (real_scene ? 1 : 0) << "\n  map_size: [30, 15, 3]\n  fix_map_origin: [0, 0, 1.5]\n"
        "  load_pcd_en: true\n  pcd_name: " << pcd.string() << "\n"
        "  virtual_ground_height: -10\n  virtual_ceil_height: 10\n"
        "  map_sliding: {enable: false}\n  ros_callback: {enable: false}\n"
        "  visualization: {enable: false, range: [30, 15, 3]}\n"
        "  point_filt_num: 1\n  intensity_thresh: -10\n"
        "  raycasting: {enable: false, p_hit: 0.9, p_occ: 0.85, p_max: 0.98, p_miss: 0.49, p_free: 0.499}\n";
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("map_config_path", config.string()),
                                rclcpp::Parameter("planner_seed", 0)});
    auto node = std::make_shared<rclcpp::Node>("topo_map_contract", options);
    node->declare_parameter("planner_seed", 0);
    GridMap map;
    map.init(*node);
    TopologyPRM prm;
    prm.setGridMap(&map);
    prm.init(*node);
    int failures = 0;
    auto check = [&](bool ok, const char* name) {
        std::cout << (ok ? "PASS " : "FAIL ") << name << '\n';
        failures += !ok;
    };
    if (real_scene) {
        const Eigen::Vector3d start(-14,-6.5,.49), goal(6.8,5.2,.49);
        const Eigen::Vector3d end = start + 8.0 * (goal-start).normalized();
        int solved = 0, path_count = 0, unsafe_edges = 0;
        for (int seed = 0; seed < 10; ++seed) {
            node->set_parameter(rclcpp::Parameter("planner_seed", seed));
            TopologyPRM search;
            search.setGridMap(&map); search.init(*node);
            std::list<GraphNode::Ptr> graph;
            std::vector<std::vector<Eigen::Vector3d>> raw, paths;
            const auto begin = std::chrono::steady_clock::now();
            search.findTopoPaths(start,end,{},{},graph,raw,paths);
            const double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
            solved += !paths.empty();
            path_count += paths.size();
            for (const auto& path : paths) {
                check((path.front()-start).norm()<1e-10 && (path.back()-end).norm()<1e-10,
                      "real PCD path retains query endpoints");
                for (size_t j=1; j<path.size(); ++j) {
                    const auto d=path[j]-path[j-1];
                    const int steps=std::max(1,static_cast<int>(std::ceil(d.norm()/.005)));
                    for (int k=0;k<=steps;++k)
                        if (map(Vector3d(path[j-1]+(double(k)/steps)*d-map.origin()))) { ++unsafe_edges; break; }
                }
            }
            std::cout << "QUERY seed=" << seed << " paths=" << paths.size() << " elapsed_ms=" << ms
                      << " reason=" << search.failureReason() << '\n';
        }
        check(solved==10, "actual PCD query connects for all ten fixed seeds");
        check(unsafe_edges==0, "actual PCD paths contain no unchecked occupied edge");
        std::cout << "SCENE solved=" << solved << "/10 paths=" << path_count << " unsafe_edges=" << unsafe_edges << '\n';
        rclcpp::shutdown();
        return failures ? 1 : 0;
    }
    Eigen::Vector3d hit;
    const Eigen::Vector3d occupied(.05, .05, 1.55), free(.15, .05, 1.55);
    check(map(Vector3d(occupied - map.origin())), "fixture voxel occupies expected world position");
    check(!map(Vector3d(free - map.origin())), "fixture free endpoint");
    check(!map.isLineFree(free, occupied), "short edge checks occupied endpoint");
    check(!map.isLineFree(occupied, free), "reverse edge checks occupied endpoint");
    check(!map.isLineFree(occupied, occupied), "zero-length occupied edge is blocked");
    check(!map.isLineFree({-.11,.05,1.55}, {.29,.05,1.55}),
          "visibility cannot skip a 0.1 m voxel");
    check(!map.isLineFree({14.9,0,1.5}, {15.01,0,1.5}), "world upper boundary is blocked");
    check(map.isLineFree({-14,-6,1}, {-13,-6,1}), "negative world coordinates remain free");
    check(map.isLineFree(free, free), "zero-length free edge remains free");
    rog_map::raycaster::RayCaster ray;
    ray.setResolution(.1);
    ray.setInput({-14.149871526242089, -4.9671511347973532, 1.0336931754556029}, {-14,-6,1});
    bool more = true;
    int visited = 0;
    while (visited++ < 20 && (more = ray.step(hit))) {}
    check(!more, "native ray terminates at a mixed-direction voxel boundary");
    if (more) { rclcpp::shutdown(); return 1; }
    std::list<GraphNode::Ptr> graph;
    std::vector<std::vector<Eigen::Vector3d>> raw, filtered;
    prm.findTopoPaths(free, occupied, {}, {}, graph, raw, filtered);
    check(filtered.empty(), "occupied endpoint cannot generate a path");
    prm.findTopoPaths({-14,-6,1}, {-13,-6,1}, {}, {}, graph, raw, filtered);
    check(!filtered.empty(), "valid query after invalid query has no stale state");
    rclcpp::shutdown();
    return failures ? 1 : 0;
}
