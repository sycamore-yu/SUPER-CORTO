#!/usr/bin/env python3
"""Direct execution scoring: sampled point geometry and complete missions."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from benchmark_geometry import SceneGeometry
from benchmark_report import score_trial


class ProtocolTests(unittest.TestCase):
    def score(self, points, scene):
        manifest = json.loads((ROOT / "experiments/manifest.json").read_text())
        events = [{"event": "mission_start", "sim_time_s": 0},
                  {"event": "plan_result", "initial": True, "success": True,
                   "elapsed_ms": 50, "sim_time_s": 0},
                  {"event": "plan_result", "initial": False, "success": False,
                   "elapsed_ms": 70, "sim_time_s": 0.05},
                  {"event": "mission_end", "origin": "collector", "reason": "goal_reached",
                   "sim_time_s": points[-1][0]}]
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory / "events.jsonl").write_text("".join(json.dumps(x) + "\n" for x in events))
            with (directory / "trajectory.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["sim_time_s", "x", "y", "z"])
                writer.writerows(points)
            return score_trial(directory, manifest, {"run_id": "synthetic", "scene": "test", "planner_seed": 0}, scene)

    def test_collision_does_not_hide_later_arrival(self):
        scene = {"bounds": [[-3, -3, 0], [3, 3, 3]], "goal": [1, 0, 1],
                 "obstacles": [{"type": "cylinder", "x": 0, "y": 0, "z": 0, "height": 3, "radius": 0.2}]}
        result = self.score([[0, -1, 0, 1], [0.05, 0, 0, 1], [0.1, 1, 0, 1]], scene)
        self.assertTrue(result["reached_goal"])
        self.assertFalse(result["success"])
        self.assertAlmostEqual(result["flight_time_s"], 0.1)
        self.assertEqual(result["planning_failures"], 1)
        self.assertEqual(result["online_replanning_ms"], 70)

    def test_point_clearance_has_no_vehicle_envelope(self):
        scene = {"bounds": [[-3, -3, 0], [3, 3, 3]], "goal": [0.4, 0, 1],
                 "obstacles": [{"type": "cylinder", "x": 0, "y": 0, "z": 0, "height": 3, "radius": 0.2}]}
        result = self.score([[0, -1, 1, 1], [0.05, 0.201, 0, 1], [0.1, 0.4, 0, 1]], scene)
        self.assertTrue(result["success"])

    def test_boundary_is_not_shrunk(self):
        scene = {"bounds": [[-3, -3, 0], [3, 3, 3]], "goal": [1, 0, 0.2],
                 "obstacles": [{"type": "cylinder", "x": 0, "y": 2, "z": 0, "height": 3, "radius": 0.2}]}
        result = self.score([[0, -1, 0, 0.2], [0.05, 0, 0, 0], [0.1, 1, 0, 0.2]], scene)
        self.assertTrue(result["success"])

    def test_out_of_bounds_does_not_hide_later_arrival(self):
        scene = {"bounds": [[-3, -3, 0], [3, 3, 3]], "goal": [1, 0, 1],
                 "obstacles": [{"type": "cylinder", "x": 0, "y": 2, "z": 0, "height": 3, "radius": 0.2}]}
        result = self.score([[0, -1, 0, 1], [0.05, 0, 0, -1], [0.1, 1, 0, 1]], scene)
        self.assertTrue(result["reached_goal"])
        self.assertFalse(result["success"])
        self.assertAlmostEqual(result["flight_time_s"], 0.1)

    def test_cylinder_distance(self):
        geometry = SceneGeometry({"obstacles": [{"type": "cylinder", "x": 0, "y": 0, "z": 0, "height": 2, "radius": 1}]})
        np.testing.assert_allclose(geometry.clearance([[0, 0, 1], [2, 0, 1], [0, 0, 3]]), [-1, 1, 1])

    def test_frozen_counts(self):
        manifest = json.loads((ROOT / "experiments/manifest.json").read_text())
        self.assertEqual(len(manifest["runs"]), 30)
        for name, count in (("sparse", 45), ("medium", 90), ("dense", 180)):
            scene = json.loads((ROOT / "experiments/scenes" / f"{name}.json").read_text())
            self.assertEqual(len(scene["obstacles"]), count)


if __name__ == "__main__":
    unittest.main()
