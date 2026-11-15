"""CORTO Table III on official SUPER simulation with a fixed PCD map."""
import json
import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    scene = value("scene")
    seed = int(value("planner_seed"))
    output = Path(value("output_dir")).resolve()
    output.mkdir(parents=True, exist_ok=True)
    scene_json = ROOT / "experiments" / "scenes" / f"{scene}.json"
    scene_pcd = scene_json.with_suffix(".pcd")
    geometry = json.loads(scene_json.read_text())
    start_pos, goal_pos = geometry["start"], geometry["goal"]
    simulator = yaml.safe_load((ROOT / "mars_uav_sim/perfect_drone_sim/config/dense.yaml").read_text())
    simulator.update({"pcd_name": str(scene_pcd),
                      "init_position": dict(zip(("x", "y", "z"), start_pos)), "init_yaw": math.pi})
    simulator_path = output / "sim_config.yaml"
    simulator_path.write_text(yaml.safe_dump(simulator, sort_keys=False))
    rog = {
        "resolution": 0.1, "inflation_resolution": 0.1, "inflation_step": 1,
        "unk_inflation_en": False, "unk_inflation_step": 0,
        "map_size": [30.0, 15.0, 3.0], "fix_map_origin": [0.0, 0.0, 1.5],
        "frontier_extraction_en": False, "virtual_ground_height": -10.0, "virtual_ceil_height": 10.0,
        "load_pcd_en": True, "pcd_name": str(scene_pcd),
        "map_sliding": {"enable": False, "threshold": 3.0},
        "esdf": {"enable": False, "resolution": 0.1, "local_update_box": [30.0, 15.0, 3.0]},
        "ros_callback": {"enable": False, "cloud_topic": "/cloud_registered",
                         "odom_topic": "/lidar_slam/odom", "odom_timeout": 2.0},
        "visualization": {"enable": value("rviz").lower() == "true", "use_dynamic_reconfigure": False,
                          "time_rate": 1.0, "frame_rate": 0, "range": [30.0, 15.0, 3.0],
                          "frame_id": "world", "pub_unknown_map_en": False},
        "intensity_thresh": -10, "point_filt_num": 1,
        "raycasting": {"enable": False, "batch_update_size": 1, "local_update_box": [30.0, 15.0, 3.0],
                       "ray_range": [0.1, 40.0], "p_min": 0.12, "p_miss": 0.49, "p_free": 0.499,
                       "p_occ": 0.85, "p_hit": 0.9, "p_max": 0.98, "unk_thresh": 1.0},
    }
    map_path = output / "map_config.yaml"
    map_path.write_text(yaml.safe_dump({"rog_map": rog}, sort_keys=False))
    parameters = {"map_config_path": str(map_path), "map_size": [30.0, 15.0, 3.0],
                  "map_origin": [-15.0, -7.5, 0.0],
                  "planner_seed": seed, "use_sim_time": True,
                  "ref.max_velocity": 10.0, "ref.max_acceleration": 5.0,
                  "nmpc.maxVel": 10.0, "nmpc.real_maxVel": 10.0,
                  "MaxPointCloud": 10000, "n_sweep": 7, "poly_degree": 3, "frame": "PTF"}
    for prefix, coordinates in (("start_pos", start_pos), ("goal_pos", goal_pos)):
        parameters.update({f"{prefix}_{axis}": float(coordinate) for axis, coordinate in zip("xyz", coordinates)})
    (output / "launch_inputs.json").write_text(json.dumps({"scene": scene, "planner_seed": seed,
                                                          "corto_parameters": parameters}, indent=2) + "\n")
    nodes = [
        Node(package="perfect_drone_sim", executable="perfect_drone_node", output="screen", prefix="vglrun -d egl",
             parameters=[{"config_name": str(simulator_path), "use_sim_time": True}]),
        Node(package="corto_ros2", executable="corto_node", output="screen", parameters=[parameters]),
    ]
    nodes.extend([
        RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="Required simulation/planner process exited"))]))
        for process in nodes[:2]
    ])
    if value("rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", str(ROOT / "launch/table3.rviz")], parameters=[{"use_sim_time": True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="medium", choices=["sparse", "medium", "dense"]),
        DeclareLaunchArgument("planner_seed", default_value="0"),
        DeclareLaunchArgument("output_dir", default_value=str(ROOT / "results/manual")),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=start),
    ])
