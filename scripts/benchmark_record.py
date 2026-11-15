#!/usr/bin/env python3
"""Record ROS data and end each trial at the first geometric terminal condition."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from benchmark_report import score_trial
from benchmark_geometry import SceneGeometry, first_terminal_on_segment


ROOT = Path(__file__).resolve().parents[1]
ODOM_FIELDS = ["sim_time_s", "wall_receive_s", "x", "y", "z", "qw", "qx", "qy", "qz",
               "vx", "vy", "vz", "wx", "wy", "wz"]
CONTROL_FIELDS = ["sim_time_s", "wall_receive_s", "body_rate_x", "body_rate_y", "body_rate_z",
                  "normalized_thrust", "unused_torque_y", "unused_torque_z"]


def stamp_seconds(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


class Recorder(Node):
    def __init__(self, manifest, run, output, scene):
        super().__init__("corto_table3_recorder")
        self.manifest, self.run, self.output = manifest, run, output
        self.started_wall = time.monotonic()
        self.mission_start = None
        self.latest_sim = 0.0
        self.ended_wall = None
        self.interrupted = False
        self.geometry = SceneGeometry(scene)
        self.goal = np.asarray(scene["goal"])
        self.previous_state = None
        self.files = {name: (output / name).open("w") for name in ("trajectory.csv", "control.csv", "events.jsonl")}
        self.odom_writer = csv.writer(self.files["trajectory.csv"])
        self.control_writer = csv.writer(self.files["control.csv"])
        self.odom_writer.writerow(ODOM_FIELDS)
        self.control_writer.writerow(CONTROL_FIELDS)
        topic = "/corto/nominal_odom" if run["mode"] == "nominal" else "/lidar_slam/odom"
        self.create_subscription(Odometry, topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, "/corto/control", self.on_control, 10000)
        self.create_subscription(String, "/corto/events", self.on_event, 10000)
        (output / "ready").write_text("subscriptions_created\n")

    def wall(self):
        return time.monotonic() - self.started_wall

    def on_odom(self, message):
        self.latest_sim = stamp_seconds(message.header.stamp)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        self.odom_writer.writerow([self.latest_sim, self.wall(), p.x, p.y, p.z,
                                   q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z])
        point = np.array([p.x, p.y, p.z])
        previous = self.previous_state
        self.previous_state = (self.latest_sim, point)
        if self.mission_start is None or self.ended_wall is not None or self.latest_sim < self.mission_start:
            return
        if not np.isfinite(point).all():
            self.finish("nonfinite_state")
            return
        start_time, start = previous if previous is not None else (self.latest_sim, point)
        if start_time < self.mission_start:
            alpha = (self.mission_start - start_time) / (self.latest_sim - start_time)
            start = start + alpha * (point - start)
            start_time = self.mission_start
        deadline = self.mission_start + self.manifest["flight_timeout_s"]
        check_time = min(self.latest_sim, deadline)
        check_point = point
        if self.latest_sim > deadline and self.latest_sim > start_time:
            check_point = start + (deadline - start_time) / (self.latest_sim - start_time) * (point - start)
        terminal = first_terminal_on_segment(self.geometry, start_time, start, check_time, check_point,
                                             self.goal, self.manifest["body_radius_m"],
                                             self.manifest["goal_tolerance_m"],
                                             self.manifest["continuous_sample_spacing_m"])
        if terminal:
            reason = terminal.pop("reason")
            self.finish(reason, evaluation_end_sim_time_s=self.latest_sim,
                        observed_position=point.tolist(), **terminal)
        elif self.latest_sim >= deadline:
            self.finish("collector_flight_timeout", terminal_time_s=deadline,
                        evaluation_end_sim_time_s=self.latest_sim)

    def on_control(self, message):
        rate, thrust = message.wrench.force, message.wrench.torque
        self.control_writer.writerow([stamp_seconds(message.header.stamp), self.wall(),
                                      rate.x, rate.y, rate.z, thrust.x, thrust.y, thrust.z])

    def append_event(self, event):
        event["wall_receive_s"] = self.wall()
        self.files["events.jsonl"].write(json.dumps(event, allow_nan=False) + "\n")
        self.files["events.jsonl"].flush()

    def on_event(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            event = {"event": "invalid_event_payload", "raw": message.data}
        self.append_event(event)
        if event.get("event") == "mission_start":
            self.mission_start = float(event["sim_time_s"])

    def finish(self, reason, **details):
        self.append_event({"event": "mission_end", "origin": "collector", "reason": reason,
                           "sim_time_s": self.latest_sim, "wall_time_s": self.wall(), **details})
        self.ended_wall = time.monotonic()

    def close(self):
        for stream in self.files.values():
            stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiments", type=Path, default=ROOT / "experiments")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest_path = args.experiments / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    run = next(x for x in manifest["runs"] if x["run_id"] == args.run_id)
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "trajectory.csv").exists():
        raise FileExistsError(f"Raw trial data already exist: {args.output}")
    scene_path = args.experiments / "scenes" / f"{run['scene']}.json"
    scene_bytes = scene_path.read_bytes()
    metadata = dict(run, manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                    protocol_version=manifest["protocol_version"], termination_policy=manifest["termination_policy"],
                    scene_sha256=hashlib.sha256(scene_bytes).hexdigest(),
                    ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0"),
                    ros_distro=os.environ.get("ROS_DISTRO"),
                    code_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    tracked_source_dirty=subprocess.run(["git", "diff", "--quiet", "HEAD", "--", ".", ":(exclude)results"], cwd=ROOT).returncode != 0,
                    started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    (args.output / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    rclpy.init()
    node = Recorder(manifest, run, args.output, json.loads(scene_bytes))
    def interrupt(_signal, _frame):
        node.interrupted = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.ended_wall is not None:
                if time.monotonic() - node.ended_wall >= manifest["post_terminal_recording_wall_s"]:
                    break
                continue
            if node.interrupted:
                exit_path = args.output / "process_exit.json"
                process_exit = json.loads(exit_path.read_text()) if exit_path.exists() else {}
                if "launch_natural_exit" in process_exit:
                    node.finish("launch_exited", launch_exit_code=process_exit["launch_natural_exit"])
                else:
                    node.finish("collector_interrupted")
            elif node.wall() >= manifest["wall_timeout_s"]:
                node.finish("collector_wall_timeout")
            elif node.mission_start is not None and node.latest_sim - node.mission_start >= manifest["flight_timeout_s"]:
                node.finish("collector_flight_timeout")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    result = score_trial(args.output, manifest, run, json.loads(scene_bytes))
    (args.output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
