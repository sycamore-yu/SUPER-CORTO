#!/usr/bin/env python3
"""Send collision, boundary and arrival states through the real ROS recorder."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]


def main():
    os.environ["ROS_DOMAIN_ID"] = "194"
    rclpy.init()
    node = rclpy.create_node("direct_recorder_transport_test")
    odom = node.create_publisher(Odometry, "/corto/odom", 10)
    control = node.create_publisher(WrenchStamped, "/corto/control", 10)
    event = node.create_publisher(String, "/corto/events", 10)
    scene = json.loads((ROOT / "experiments/scenes/sparse.json").read_text())
    cylinder = next(x for x in scene["obstacles"] if x["type"] == "cylinder")
    points = [scene["start"], scene["start"],
              [cylinder["x"], cylinder["y"], cylinder["z"] + cylinder["height"] / 2],
              [-14.0, -6.5, -1.0], scene["start"], scene["goal"]]
    with tempfile.TemporaryDirectory(prefix="corto_direct_recorder_") as directory:
        output = Path(directory)
        process = subprocess.Popen([sys.executable, str(ROOT / "scripts/benchmark_record.py"),
                                    "--run-id", "direct_sparse_00", "--output", str(output)],
                                   cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            deadline = time.monotonic() + 15
            while not all(pub.get_subscription_count() for pub in (odom, control, event)):
                rclpy.spin_once(node, timeout_sec=0.05)
                assert time.monotonic() < deadline, "Subscriptions not discovered"
                assert process.poll() is None, process.communicate()[0]
            event.publish(String(data=json.dumps({"event": "mission_start", "sim_time_s": 0})))
            time.sleep(0.1)
            for i, point in enumerate(points):
                message = Odometry()
                message.header.stamp.nanosec = i * 50_000_000
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = point
                message.pose.pose.orientation.z = 1.0
                command = WrenchStamped()
                command.header = message.header
                command.wrench.torque.x = 0.6
                if i == len(points) - 1:
                    # The terminal odometry frame may arrive before its control.
                    odom.publish(message)
                    time.sleep(0.05)
                    control.publish(command)
                else:
                    control.publish(command)
                    time.sleep(0.02)
                    odom.publish(message)
                time.sleep(0.1)
                if i == 3:
                    time.sleep(0.4)
                    assert process.poll() is None, "Collision/boundary interrupted the mission"
                    event.publish(String(data=json.dumps({"event": "plan_result", "initial": False,
                                                          "success": False, "elapsed_ms": 70, "sim_time_s": 0.15})))
            stdout, _ = process.communicate(timeout=10)
            assert process.returncode == 0, stdout
            with (output / "trajectory.csv").open() as stream:
                states = list(csv.DictReader(stream))
            with (output / "control.csv").open() as stream:
                controls = list(csv.DictReader(stream))
            events = [json.loads(x) for x in (output / "events.jsonl").read_text().splitlines()]
            summary = json.loads((output / "summary.json").read_text())
            assert len(states) == 6 and len(controls) == 6
            assert summary["termination_reason"] == "goal_reached", summary
            assert summary["reached_goal"] and not summary["success"], summary
            assert summary["collision"] and summary["out_of_bounds"], summary
            assert summary["flight_time_s"] == 0.25, summary
            assert summary["replanning_calls"] == 1 and summary["planning_failures"] == 1, summary
            assert sum(x["event"] == "collision" for x in events) == 1
            assert sum(x["event"] == "out_of_bounds" for x in events) == 1
            print(json.dumps({"passed": True, "states": len(states), "controls": len(controls),
                              "collision_continued": True, "out_of_bounds_continued": True,
                              "arrival_recorded": True, "safe_success": summary["success"]}))
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
