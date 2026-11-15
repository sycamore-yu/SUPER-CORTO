#!/usr/bin/env python3
"""Synthetic ROS transport smoke; its temporary data are not experiment trials."""
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
    os.environ["ROS_DOMAIN_ID"] = "189"
    rclpy.init()
    node = rclpy.create_node("synthetic_recorder_transport_test")
    odom = node.create_publisher(Odometry, "/corto/nominal_odom", 10)
    control = node.create_publisher(WrenchStamped, "/corto/control", 10)
    event = node.create_publisher(String, "/corto/events", 10)
    scene = json.loads((ROOT / "experiments/scenes/sparse.json").read_text())
    cylinder = next(x for x in scene["obstacles"] if x["type"] == "cylinder")
    with tempfile.TemporaryDirectory(prefix="corto_synthetic_recorder_") as directory:
        output = Path(directory)
        process = subprocess.Popen([sys.executable, str(ROOT / "scripts/benchmark_record.py"),
                                    "--run-id", "nominal_sparse_00", "--output", str(output)],
                                   cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if all(pub.get_subscription_count() for pub in (odom, control, event)):
                    break
                if process.poll() is not None:
                    raise RuntimeError(process.communicate()[0])
            else:
                raise RuntimeError("Recorder subscriptions were not discovered")
            event.publish(String(data=json.dumps({"event": "mission_start", "sim_time_s": 0, "wall_time_s": 0})))
            for i in range(6):
                message = Odometry()
                message.header.stamp.nanosec = i * 50_000_000
                message.pose.pose.position.x = -14 + i * 0.01
                message.pose.pose.position.y = -6.5
                message.pose.pose.position.z = 0.49
                if i == 4:
                    message.pose.pose.position.x = cylinder["x"]
                    message.pose.pose.position.y = cylinder["y"]
                    message.pose.pose.position.z = 1.5
                elif i == 5:
                    message.pose.pose.position.z = 100.0
                message.pose.pose.orientation.z = 1.0
                odom.publish(message)
                command = WrenchStamped()
                command.header = message.header
                command.wrench.torque.x = 0.6
                control.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)
            event.publish(String(data=json.dumps({"event": "plan_result", "initial": False,
                                                  "success": False, "elapsed_ms": 0.03, "sim_time_s": 0.25})))
            stdout, _ = process.communicate(timeout=15)
            if process.returncode:
                raise RuntimeError(stdout)
            with (output / "trajectory.csv").open() as stream:
                states = list(csv.DictReader(stream))
            with (output / "control.csv").open() as stream:
                controls = list(csv.DictReader(stream))
            events = [json.loads(x) for x in (output / "events.jsonl").read_text().splitlines()]
            summary = json.loads((output / "summary.json").read_text())
            assert len(states) == 6, len(states)
            assert len(controls) == 6, len(controls)
            assert len(events) == 3, events
            assert summary["termination_reason"] == "collision", summary
            assert summary["first_body_collision_time_s"] < 0.2, summary
            assert summary["flight_observed_duration_s"] == 0.2, summary
            assert summary["after_terminal_plan_results"] == 1, summary
            assert summary["planning_calls"] == 0, summary
            assert summary["success"] is False, summary
            print(json.dumps({"synthetic_test": True, "passed": True, "states": len(states),
                              "controls": len(controls), "events": len(events)}))
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
