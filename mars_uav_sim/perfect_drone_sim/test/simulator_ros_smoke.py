#!/usr/bin/env python3
"""Exercise both PerfectDrone modes against actual ROS 2, clock and MARSIM output."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
from mars_quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path as NavPath


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("perfect_drone_smoke")
    rows, clocks, lidar, motors = [], [], [], []
    transforms, markers, paths = [], [], []
    started = time.monotonic()

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        rows.append({
            "t": stamp_seconds(msg.header.stamp),
            "p": [p.x, p.y, p.z], "q_xyzw": [q.x, q.y, q.z, q.w],
            "v": [v.x, v.y, v.z], "omega": [w.x, w.y, w.z],
            "wall_s": time.monotonic() - started,
        })

    subscriptions = [
        node.create_subscription(Odometry, "/lidar_slam/odom", on_odom, qos_profile_sensor_data),
        node.create_subscription(Clock, "/clock",
                                 lambda msg: clocks.append(stamp_seconds(msg.clock)),
                                 qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/cloud_registered",
                                 lambda msg: lidar.append([stamp_seconds(msg.header.stamp),
                                                           msg.width * msg.height]),
                                 qos_profile_sensor_data),
        node.create_subscription(Float64MultiArray, "/simulator/motor_rpm",
                                 lambda msg: motors.append(list(msg.data)),
                                 qos_profile_sensor_data),
        node.create_subscription(TFMessage, "/tf",
                                 lambda msg: transforms.extend(
                                     t.child_frame_id for t in msg.transforms),
                                 qos_profile_sensor_data),
        node.create_subscription(Marker, "/robot",
                                 lambda msg: markers.append(msg.mesh_resource),
                                 qos_profile_sensor_data),
        node.create_subscription(NavPath, "/path",
                                 lambda msg: paths.append(len(msg.poses)),
                                 qos_profile_sensor_data),
    ]
    ctbr_pub = node.create_publisher(WrenchStamped, "/corto/control", 1)
    position_pub = node.create_publisher(PositionCommand, "/planning/pos_cmd", 1)
    log = (output / f"{args.mode}.log").open("w")
    proc = subprocess.Popen(
        ["vglrun", "-d", "egl", args.simulator, "--ros-args",
         "-p", f"config_name:={args.config}", "-p", f"dynamics_mode:={args.mode}",
         "-p", "use_sim_time:=true"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 30.0
        while not rows and time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rows:
            raise RuntimeError(f"No odometry; inspect {output / (args.mode + '.log')}")
        initial = rows[0]["p"]
        first_time = rows[0]["t"]
        first_wall = time.monotonic()
        while time.monotonic() - first_wall < 4.0 and proc.poll() is None:
            elapsed = rows[-1]["t"] - first_time
            if args.mode == "so3":
                msg = WrenchStamped()
                msg.wrench.torque.x = 0.6 if elapsed < 1.0 else 0.63
                msg.wrench.force.z = 0.0 if elapsed < 1.0 else 0.3
                ctbr_pub.publish(msg)
            else:
                msg = PositionCommand()
                msg.position.x, msg.position.y, msg.position.z = initial
                msg.position.x += 0.5
                msg.yaw = 0.2
                position_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
        if proc.poll() is not None:
            raise RuntimeError(f"Simulator exited with {proc.returncode}")

        result = {
            "mode": args.mode, "odometry_count": len(rows),
            "clock_count": len(clocks), "lidar_count": len(lidar),
            "lidar_nonempty_count": sum(n > 0 for _, n in lidar),
            "motor_count": len(motors),
            "tf_drone_count": transforms.count("perfect_drone"),
            "robot_marker_count": len(markers), "path_count": len(paths),
            "simulation_span_s": rows[-1]["t"] - first_time,
            "wall_span_s": rows[-1]["wall_s"] - rows[0]["wall_s"],
            "initial_position_m": initial, "final": rows[-1],
            "all_states_finite": all(math.isfinite(x) for r in rows
                                     for field in ["p", "q_xyzw", "v", "omega"]
                                     for x in r[field]),
            "clock_monotonic": all(a < b for a, b in zip(clocks, clocks[1:])),
        }
        if args.mode == "so3":
            result["input_response_passed"] = (
                rows[-1]["omega"][2] > 0.25 and rows[-1]["p"][2] - initial[2] > 0.1
                and len(motors) > 50)
        else:
            target = [initial[0] + 0.5, initial[1], initial[2]]
            result["input_response_passed"] = math.dist(rows[-1]["p"], target) < 1e-8
        result["passed"] = (
            result["all_states_finite"] and result["clock_monotonic"]
            and result["input_response_passed"] and result["lidar_nonempty_count"] > 2
            and result["tf_drone_count"] > 50 and len(markers) > 50 and len(paths) > 2
            and result["simulation_span_s"] > 3.0)
        with (output / f"{args.mode}_odom.jsonl").open("w") as stream:
            for row in rows:
                stream.write(json.dumps(row) + "\n")
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    result["exit_code"] = proc.returncode
    result["passed"] = result["passed"] and proc.returncode == 0
    (output / f"{args.mode}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise RuntimeError("Simulator integration/exit smoke failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["perfect", "so3"], required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--simulator", required=True)
    parser.add_argument("--output", required=True)
    run(parser.parse_args())

