#!/usr/bin/env python3
"""Observe CORTO directly; collision and boundary events never interrupt control."""
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
from std_msgs.msg import String

from benchmark_geometry import SceneGeometry
from benchmark_report import score_trial

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
        self.planner_end = None
        self.interrupted = False
        self.first_events = set()
        self.geometry = SceneGeometry(scene)
        self.goal = np.asarray(scene["goal"])
        self.files = {name: (output / name).open("w") for name in ("trajectory.csv", "control.csv", "events.jsonl")}
        self.odom_writer = csv.writer(self.files["trajectory.csv"])
        self.control_writer = csv.writer(self.files["control.csv"])
        self.odom_writer.writerow(ODOM_FIELDS)
        self.control_writer.writerow(CONTROL_FIELDS)
        self.create_subscription(Odometry, "/corto/odom", self.on_odom, 1000)
        self.create_subscription(WrenchStamped, "/corto/control", self.on_control, 1000)
        self.create_subscription(String, "/corto/events", self.on_event, 1000)
        (output / "ready").write_text("subscriptions_created\n")

    def wall(self):
        return time.monotonic() - self.started_wall

    def on_odom(self, message):
        self.latest_sim = stamp_seconds(message.header.stamp)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        state = [p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z]
        self.odom_writer.writerow([self.latest_sim, self.wall(), *state])
        if self.mission_start is None or self.ended_wall is not None:
            return
        if not np.isfinite(state).all():
            self.finish("nonfinite_state")
            return
        point = np.array([p.x, p.y, p.z])
        conditions = {"collision": self.geometry.clearance(point)[0] <= 0,
                      "out_of_bounds": self.geometry.out_of_bounds(point)[0]}
        for name, active in conditions.items():
            if active and name not in self.first_events:
                self.first_events.add(name)
                self.append_event({"event": name, "origin": "collector", "sim_time_s": self.latest_sim,
                                   "position": point.tolist()})
        if np.linalg.norm(point - self.goal) < self.manifest["goal_tolerance_m"]:
            self.finish("goal_reached")
        elif self.latest_sim - self.mission_start >= self.manifest["flight_timeout_s"] - 1e-9:
            self.finish("collector_flight_timeout")

    def on_control(self, message):
        rate, thrust = message.wrench.force, message.wrench.torque
        self.control_writer.writerow([stamp_seconds(message.header.stamp), self.wall(),
                                      rate.x, rate.y, rate.z, thrust.x, thrust.y, thrust.z])

    def append_event(self, event):
        event["wall_receive_s"] = self.wall()
        self.files["events.jsonl"].write(json.dumps(event, allow_nan=False) + "\n")
        self.files["events.jsonl"].flush()

    def on_event(self, message):
        event = json.loads(message.data)
        self.append_event(event)
        if event["event"] == "mission_start":
            self.mission_start = float(event["sim_time_s"])
        elif event["event"] == "mission_end":
            self.planner_end = event

    def finish(self, reason):
        self.append_event({"event": "mission_end", "origin": "collector", "reason": reason,
                           "sim_time_s": self.latest_sim})
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
                    protocol_version=manifest["protocol_version"], execution=manifest["execution"],
                    scene_sha256=hashlib.sha256(scene_bytes).hexdigest(),
                    ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0"), ros_distro=os.environ.get("ROS_DISTRO"),
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
        while rclpy.ok() and node.ended_wall is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.ended_wall is not None:
                break
            # ROS topics can be delivered in a different order. Retain the final state.
            if node.planner_end is not None and node.latest_sim >= node.planner_end["sim_time_s"] - 1e-9:
                node.finish(node.planner_end["reason"])
            elif node.interrupted:
                node.finish("launch_exited")
            elif node.wall() >= manifest["wall_timeout_s"]:
                node.finish("collector_wall_timeout")
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
