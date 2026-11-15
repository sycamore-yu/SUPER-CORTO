#!/usr/bin/env python3
"""Uninflated cylinder/tube geometry used by Table III generation and scoring."""
import math

import numpy as np
from scipy.spatial import cKDTree


class SceneGeometry:
    def __init__(self, scene, ring_samples=4096):
        self.scene = scene
        self.bounds = np.asarray(scene.get("bounds", [[-15, -7.5, 0], [15, 7.5, 3]]), dtype=float)
        self.cylinders = [x for x in scene["obstacles"] if x["type"] == "cylinder"]
        self.rings = []
        angle = np.arange(ring_samples) * (2 * math.pi / ring_samples)
        for obs in scene["obstacles"]:
            if obs["type"] != "circle":
                continue
            r1, r2, yaw = obs["radius1"], obs["radius2"], obs["theta"]
            local_y, local_z = r1 * np.cos(angle), r2 * np.sin(angle)
            points = np.column_stack((-math.sin(yaw) * local_y + obs["x"],
                                      math.cos(yaw) * local_y + obs["y"],
                                      local_z + obs["z"]))
            # Every point of the continuous ellipse is this close to a sample.
            error = math.pi * max(r1, r2) / ring_samples
            self.rings.append((cKDTree(points), obs["thick"] / 2, error))
        self.error_bound_m = max((x[2] for x in self.rings), default=0.0)

    def clearance(self, points):
        """Return center-to-solid signed distance; rings have a recorded error bound."""
        points = np.atleast_2d(points).astype(float)
        result = np.full(len(points), np.inf)
        for obs in self.cylinders:
            radial = np.hypot(points[:, 0] - obs["x"], points[:, 1] - obs["y"]) - obs["radius"]
            vertical = np.abs(points[:, 2] - obs["z"] - obs["height"] / 2) - obs["height"] / 2
            outside = np.hypot(np.maximum(radial, 0), np.maximum(vertical, 0))
            signed = outside + np.minimum(np.maximum(radial, vertical), 0)
            result = np.minimum(result, signed)
        for tree, radius, _ in self.rings:
            distance, _ = tree.query(points, workers=1)
            result = np.minimum(result, distance - radius)
        return result

    def lower_clearance(self, points):
        return self.clearance(points) - self.error_bound_m

    def out_of_bounds(self, points):
        points = np.atleast_2d(points)
        return np.any((points < self.bounds[0]) | (points >= self.bounds[1]), axis=1)
