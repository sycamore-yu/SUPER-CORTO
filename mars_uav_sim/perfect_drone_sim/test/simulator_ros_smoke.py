#!/usr/bin/env python3
"""Verify that MARSIM renders the exact CORTO state without advancing it."""
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
from nav_msgs.msg import Odometry, Path as NavPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


def state(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    return [p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z]


def stamp(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("corto_state_render_test")
    source = node.create_publisher(Odometry, "/corto/odom", rclpy.qos.QoSProfile(depth=100,
                                   durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    clock = node.create_publisher(Clock, "/clock", 10)
    received, sent, lidar, transforms, markers, paths = {}, {}, [], {}, [], []
    subscriptions = [
        node.create_subscription(Odometry, "/lidar_slam/odom", lambda msg: received.update({stamp(msg): state(msg)}), qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/cloud_registered", lambda msg: lidar.append((stamp(msg), msg.width * msg.height)), qos_profile_sensor_data),
        node.create_subscription(TFMessage, "/tf", lambda msg: transforms.update({stamp(t): t for t in msg.transforms if t.child_frame_id == "perfect_drone"}), qos_profile_sensor_data),
        node.create_subscription(Marker, "/robot", lambda msg: markers.append(msg), qos_profile_sensor_data),
        node.create_subscription(NavPath, "/path", lambda msg: paths.append(msg), qos_profile_sensor_data),
    ]
    with (output / "renderer.log").open("w") as log:
        process = subprocess.Popen(["vglrun", "-d", "egl", args.simulator, "--ros-args",
                                    "-p", f"config_name:={args.config}", "-p", "use_sim_time:=true"],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while source.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                assert process.poll() is None, "Renderer exited before state discovery"
            assert source.get_subscription_count() > 0, "Renderer does not consume CORTO Odometry"
            for index in range(61):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = "world", "body"
                ns = index * 50000000
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1000000000)
                message.pose.pose.position.x = -14.0 + index * 0.02
                message.pose.pose.position.y, message.pose.pose.position.z = -6.5, 0.49
                q = message.pose.pose.orientation
                q.w, q.x, q.y, q.z = math.sqrt(0.86), 0.2, -0.1, 0.3
                v, w = message.twist.twist.linear, message.twist.twist.angular
                v.x, v.y, v.z = 0.4, -0.2, 0.1
                w.x, w.y, w.z = 0.5, -0.3, 0.2
                sent[ns] = state(message)
                clock.publish(Clock(clock=message.header.stamp))
                source.publish(message)
                until = time.monotonic() + 0.05
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.01)
            until = time.monotonic() + 1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert received.keys() == sent.keys(), (len(received), len(sent))
            error = max(abs(a - b) for key in sent for a, b in zip(received[key], sent[key]))
            assert error == 0, error
            assert transforms, "No drone TF"
            for key, transform in transforms.items():
                t, q = transform.transform.translation, transform.transform.rotation
                assert [t.x, t.y, t.z, q.w, q.x, q.y, q.z] == sent[key][:7]
            assert any(count > 0 for _, count in lidar), "LiDAR is empty"
            assert all(key in sent for key, _ in lidar), "LiDAR has an independent time source"
            assert markers and paths, "Missing visualization"
            assert node.count_publishers("/clock") == 1, "Renderer publishes a second clock"
            assert node.count_subscribers("/corto/control") == 0, "Renderer consumes control commands"
            result = {"passed": True, "state_count": len(received), "max_state_error": error,
                      "tf_count": len(transforms), "lidar_count": len(lidar),
                      "nonempty_lidar_count": sum(n > 0 for _, n in lidar),
                      "marker_count": len(markers), "path_count": len(paths)}
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            node.destroy_node()
            rclpy.shutdown()
    assert process.returncode == 0, process.returncode
    result["exit_code"] = process.returncode
    (output / "renderer.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--simulator", required=True)
    parser.add_argument("--output", required=True)
    run(parser.parse_args())
