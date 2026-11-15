#!/usr/bin/env python3
"""Acceptance checks for geometry, crossing detection and failure-preserving metrics."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from benchmark_geometry import SceneGeometry, first_terminal_on_segment
from benchmark_report import first_goal_entry, score_trial


class ProtocolTests(unittest.TestCase):
    def test_finite_cylinder_signed_distance(self):
        geometry = SceneGeometry({"obstacles": [{"type": "cylinder", "x": 0, "y": 0, "z": 0, "height": 2, "radius": 1}]})
        actual = geometry.clearance([[0, 0, 1], [2, 0, 1], [0, 0, 3], [2, 0, 3]])
        np.testing.assert_allclose(actual, [-1, 1, 1, np.sqrt(2)])

    def test_ellipse_tube_geometry_and_error_bound(self):
        ring = {"type": "circle", "x": 0, "y": 0, "z": 1, "radius1": 1, "radius2": 0.5, "theta": 0, "thick": 0.2}
        geometry = SceneGeometry({"obstacles": [ring]})
        np.testing.assert_allclose(geometry.clearance([[0, 1, 1], [0.3, 1, 1], [0, 0, 1]]), [-0.1, 0.2, 0.4], atol=1e-12)
        self.assertLess(geometry.error_bound_m, 0.001)

    def test_goal_entry_between_samples(self):
        result = first_goal_entry(np.array([0, 2]), np.array([[-1, 0, 1], [1, 0, 1]]), np.array([1, 0, 1]), 0.2)
        self.assertAlmostEqual(result[1], 1.8)
        np.testing.assert_allclose(result[2], [0.8, 0, 1])

    def test_swept_collision_is_failure_even_when_source_says_goal(self):
        manifest = {"collision_model": "ego_rotor_enclosing_sphere", "body_radius_m": 0.322,
                    "original_reference_radius_m": 0.1, "goal_tolerance_m": 0.2,
                    "continuous_sample_spacing_m": 0.005}
        scene = {"bounds": [[-3, -3, 0], [3, 3, 3]], "goal": [1, 0, 1],
                 "obstacles": [{"type": "cylinder", "x": 0, "y": 0, "z": 0, "height": 3, "radius": 0.2}]}
        events = [{"event": "mission_start", "sim_time_s": 0},
                  {"event": "plan_result", "initial": True, "success": True, "elapsed_ms": 50, "sim_time_s": 0},
                  {"event": "plan_result", "initial": False, "success": False, "elapsed_ms": 70, "sim_time_s": 0.1},
                  {"event": "mission_end", "sim_time_s": 2, "reason": "goal_reached"}]
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory / "events.jsonl").write_text("".join(json.dumps(x) + "\n" for x in events))
            with (directory / "trajectory.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["sim_time_s", "x", "y", "z"])
                writer.writerow([0, -1, 0, 1])
                writer.writerow([2, 1, 0, 1])
            result = score_trial(directory, manifest, {"run_id": "synthetic_unit_test", "mode": "nominal", "scene": "test", "planner_seed": 0}, scene)
        self.assertTrue(result["reached_goal"])
        self.assertTrue(result["center_collision"])
        self.assertTrue(result["body_collision"])
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_calls"], 2)
        self.assertEqual(result["planning_failures"], 1)
        self.assertEqual(result["online_replanning_ms"], 70)
        self.assertAlmostEqual(result["flight_time_s"], 1.8)
        self.assertLess(result["first_body_collision_time_s"], 1)

    def test_first_collision_terminates_before_goal_and_late_planning(self):
        scene = {"bounds": [[-3, -3, 0], [3, 3, 3]], "goal": [1, 0, 1],
                 "obstacles": [{"type": "cylinder", "x": 0, "y": 0, "z": 0, "height": 3, "radius": 0.2}]}
        manifest = {"collision_model": "ego_rotor_enclosing_sphere", "body_radius_m": 0.322,
                    "original_reference_radius_m": 0.1, "goal_tolerance_m": 0.2,
                    "continuous_sample_spacing_m": 0.005}
        terminal = first_terminal_on_segment(SceneGeometry(scene), 0, [-1, 0, 1], 2, [1, 0, 1],
                                             scene["goal"], 0.322, 0.2)
        self.assertEqual(terminal.pop("reason"), "collision")
        self.assertLess(terminal["terminal_time_s"], 0.5)
        np.testing.assert_allclose(terminal["first_contact_position"], [-0.522, 0, 1], atol=0.006)
        events = [{"event": "mission_start", "sim_time_s": 0},
                  {"event": "plan_result", "initial": False, "success": False, "elapsed_ms": 70, "sim_time_s": 0.2},
                  {"event": "plan_result", "initial": False, "success": False, "elapsed_ms": 0.04, "sim_time_s": 0.8},
                  {"event": "mission_end", "origin": "collector", "reason": "collision",
                   "sim_time_s": 2, "evaluation_end_sim_time_s": 2, **terminal},
                  {"event": "plan_result", "initial": False, "success": False, "elapsed_ms": 0.03, "sim_time_s": 2.1}]
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory / "events.jsonl").write_text("".join(json.dumps(x) + "\n" for x in events))
            with (directory / "trajectory.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["sim_time_s", "x", "y", "z"])
                writer.writerows([[0, -1, 0, 1], [2, 1, 0, 1], [2.1, 100, 0, 1]])
            result = score_trial(directory, manifest, {"run_id": "synthetic_first_terminal_test"}, scene)
        self.assertFalse(result["reached_goal"])
        self.assertTrue(result["body_collision"])
        self.assertFalse(result["body_out_of_bounds"])
        self.assertEqual(result["planning_calls"], 1)
        self.assertEqual(result["planning_failures"], 1)
        self.assertEqual(result["online_replanning_ms"], 70)
        self.assertEqual(result["after_terminal_plan_results"], 2)
        self.assertEqual(result["flight_observed_duration_s"], 2)

    def test_frozen_counts_and_nested_scenes(self):
        root = Path(__file__).resolve().parent
        manifest = json.loads((root / "manifest.json").read_text())
        self.assertEqual(len(manifest["runs"]), 60)
        previous = []
        for name, count in (("sparse", 45), ("medium", 90), ("dense", 180)):
            scene = json.loads((root / "scenes" / f"{name}.json").read_text())
            self.assertEqual(len(scene["obstacles"]), count)
            self.assertEqual(scene["obstacles"][:len(previous)], previous)
            self.assertEqual(sum(x["type"] == "circle" for x in scene["obstacles"]), count // 5)
            previous = scene["obstacles"]


if __name__ == "__main__":
    unittest.main()
