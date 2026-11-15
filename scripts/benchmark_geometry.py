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


def densify(times, positions, max_spacing=0.005):
    """Linearly interpolate each observed segment; preserve its observed timestamps."""
    times, positions = np.asarray(times), np.asarray(positions)
    out_times, out_positions = [], []
    for i in range(len(times) - 1):
        steps = max(1, int(math.ceil(np.linalg.norm(positions[i + 1] - positions[i]) / max_spacing)))
        alpha = np.arange(steps) / steps
        out_times.extend(times[i] + alpha * (times[i + 1] - times[i]))
        out_positions.extend(positions[i] + alpha[:, None] * (positions[i + 1] - positions[i]))
    if len(times):
        out_times.append(times[-1])
        out_positions.append(positions[-1])
    return np.asarray(out_times), np.asarray(out_positions)


def first_terminal_on_segment(geometry, start_time, start, end_time, end, goal,
                              body_radius, goal_radius, spacing=0.005):
    """Find the first terminal condition on one observed, linearly swept segment."""
    start, end, goal = np.asarray(start), np.asarray(end), np.asarray(goal)
    delta = end - start
    times, points = densify([start_time, end_time], [start, end], spacing)
    clearance = geometry.clearance(points)
    margin = geometry.error_bound_m + spacing / 2
    candidates = []
    collision = np.flatnonzero(clearance - margin <= body_radius)
    if len(collision):
        index = int(collision[0])
        candidates.append((float(times[index]), 0, "collision", points[index]))
    lower, upper = geometry.bounds[0] + body_radius, geometry.bounds[1] - body_radius
    for axis in range(3):
        if start[axis] <= lower[axis] or start[axis] >= upper[axis]:
            candidates.append((start_time, 1, "out_of_bounds", start))
            break
        boundary = lower[axis] if delta[axis] < 0 else upper[axis]
        if delta[axis] != 0:
            alpha = (boundary - start[axis]) / delta[axis]
            if 0 <= alpha <= 1:
                candidates.append((start_time + alpha * (end_time - start_time), 1,
                                   "out_of_bounds", start + alpha * delta))
    offset = start - goal
    a, b = float(delta @ delta), 2 * float(offset @ delta)
    c = float(offset @ offset) - goal_radius * goal_radius
    if c < 0:
        candidates.append((start_time, 2, "goal_reached", start))
    elif a > 0 and b * b - 4 * a * c > 0:
        alpha = (-b - math.sqrt(b * b - 4 * a * c)) / (2 * a)
        if 0 <= alpha < 1:
            candidates.append((start_time + alpha * (end_time - start_time), 2,
                               "goal_reached", start + alpha * delta))
    if not candidates:
        return None
    terminal_time, _, reason, position = min(candidates, key=lambda x: (x[0], x[1]))
    details = {"reason": reason, "terminal_time_s": float(terminal_time),
               "terminal_position": position.tolist(), "clearance_error_bound_m": margin}
    if reason in ("collision", "out_of_bounds"):
        details.update(first_contact_time_s=float(terminal_time), first_contact_position=position.tolist())
    return details
