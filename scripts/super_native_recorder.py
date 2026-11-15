#!/usr/bin/env python3
"""Use SUPER's original runner, but record collisions instead of stopping.

The only replacement is the observation callback. Planning, trajectory
optimization, actuator interface, and PerfectDrone are unchanged.
"""
from pathlib import Path
import sys
import time

import numpy as np

SUPER = Path("/home/tong/tongworkspace/projects/aerial/SUPER")
sys.path.insert(0, str(SUPER / "scripts"))
import run_super_baseline as source


class FullMissionRecorder(source.Recorder):
    def odom(self, message):
        self.odom_count += 1
        if self.first_odom_wall is None:
            self.first_odom_wall = time.monotonic()
        if self.start_time is None:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        t = stamp - self.start_time
        p, v = message.pose.pose.position, message.twist.twist.linear
        state = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        if stamp < self.start_time:
            self.pre_goal_rows.append([stamp, self.start_time, t, *state])
            return
        if not np.isfinite(state).all():
            self.reason = "nonfinite_state"
            return
        clearance = float(self.tree.query(state[:3])[0])
        self.rows.append([t, *state, *self.last_acc, clearance])
        if np.linalg.norm(state[:3] - self.trial["goal"]) < self.trial["goal_radius"]:
            self.reason = "goal_reached"
        elif t >= self.trial["timeout_s"]:
            self.reason = "timeout"


source.Recorder = FullMissionRecorder

if __name__ == "__main__":
    source.main()
