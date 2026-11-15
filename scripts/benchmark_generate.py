#!/usr/bin/env python3
"""Freeze reconstructed Table III scenes and the complete trial list before flight."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random

import numpy as np

from benchmark_geometry import SceneGeometry


ROOT = Path(__file__).resolve().parents[1]
COUNTS = {"sparse": 45, "medium": 90, "dense": 180}
START = [-14.0, -6.5, 0.49]
GOAL = [6.8, 5.2, 0.49]


def make_obstacles(template, seed):
    rng = random.Random(seed)
    pools = {kind: [x for x in template["obstacles"] if x["type"] == kind]
             for kind in ("circle", "cylinder")}
    result = []
    for index in range(180):
        kind = "circle" if index % 5 == 0 else "cylinder"
        source = rng.choice(pools[kind])
        obs = dict(source)
        # Preserve official shape dimensions. Placement is a declared reconstruction.
        radius = max(obs["radius1"], obs["radius2"]) + obs["thick"] / 2 if kind == "circle" else obs["radius"]
        while True:
            obs["x"] = rng.uniform(-15 + radius, 15 - radius)
            obs["y"] = rng.uniform(-7.5 + radius, 7.5 - radius)
            if kind == "circle":
                obs["z"] = rng.uniform(obs["radius2"] + obs["thick"] / 2,
                                       3 - obs["radius2"] - obs["thick"] / 2)
                obs["theta"] = rng.uniform(-math.pi, math.pi)
            candidate = SceneGeometry({"obstacles": [obs]}, ring_samples=1024)
            if np.min(candidate.lower_clearance([START, GOAL])) >= 0.6:
                break
        obs["id"] = index
        result.append(obs)
    return result


def voxel_points(scene):
    resolution = scene["resolution"]
    origin = np.asarray(scene["bounds"][0])
    cells = np.ceil(np.asarray(scene["map_size"]) / resolution).astype(int)
    occupied = set()
    for obstacle in scene["obstacles"]:
        if obstacle["type"] == "cylinder":
            ext = np.array([obstacle["radius"], obstacle["radius"], obstacle["height"] / 2])
            center = np.array([obstacle["x"], obstacle["y"], obstacle["z"] + obstacle["height"] / 2])
        else:
            half = obstacle["thick"] / 2
            ext = np.array([abs(math.sin(obstacle["theta"])) * obstacle["radius1"] + half,
                            abs(math.cos(obstacle["theta"])) * obstacle["radius1"] + half,
                            obstacle["radius2"] + half])
            center = np.array([obstacle["x"], obstacle["y"], obstacle["z"]])
        lo = np.maximum(0, np.floor((center - ext - origin) / resolution).astype(int))
        hi = np.minimum(cells, np.ceil((center + ext - origin) / resolution).astype(int))
        indices = np.stack(np.meshgrid(*(np.arange(a, b) for a, b in zip(lo, hi)), indexing="ij"), axis=-1).reshape(-1, 3)
        points = origin + (indices + 0.5) * resolution
        # PCD contains uninflated solid occupancy. ROG-Map owns planner inflation.
        inside = SceneGeometry({"obstacles": [obstacle]}).clearance(points) <= 0
        occupied.update(map(tuple, indices[inside].tolist()))
    indices = np.asarray(sorted(occupied), dtype=np.int32)
    return (origin + (indices + 0.5) * resolution).astype("<f4")


def write_pcd(path, points):
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n")
    with path.open("wb") as stream:
        stream.write(header.encode())
        stream.write(points.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=20261115)
    parser.add_argument("--source", type=Path, default=ROOT / "vendor/corto/src/planner/src/medium_obstacles.json")
    parser.add_argument("--output", type=Path, default=ROOT / "experiments")
    args = parser.parse_args()
    source_bytes = args.source.read_bytes()
    source = json.loads(source_bytes)
    obstacles = make_obstacles(source, args.seed)
    target = args.output / "scenes"
    target.mkdir(parents=True, exist_ok=True)
    manifests = []
    for name, count in COUNTS.items():
        scene = {"schema_version": 1, "name": name, "reconstructed": True,
                 "generation_seed": args.seed, "map_size": [30, 15, 3], "bounds": [[-15, -7.5, 0], [15, 7.5, 3]],
                 "resolution": 0.1, "start": START, "goal": GOAL,
                 "obstacle_count": count, "density_per_m2": count / 450,
                 "source_commit": "e05a2e60f78d9ae4425981c1343913c3c67d3bb3",
                 "source_json_sha256": hashlib.sha256(source_bytes).hexdigest(),
                 "obstacles": obstacles[:count]}
        json_path, pcd_path = target / f"{name}.json", target / f"{name}.pcd"
        json_path.write_text(json.dumps(scene, indent=2) + "\n")
        points = voxel_points(scene)
        write_pcd(pcd_path, points)
        info = {"scene": name, "scene_json": f"scenes/{name}.json", "scene_pcd": f"scenes/{name}.pcd",
                "scene_sha256": hashlib.sha256(json_path.read_bytes()).hexdigest(),
                "pcd_sha256": hashlib.sha256(pcd_path.read_bytes()).hexdigest(),
                "point_count": len(points), "obstacle_count": count,
                "circle_count": count // 5, "cylinder_count": count * 4 // 5}
        manifests.append(info)
        print(json.dumps(info))
    manifest = {"schema_version": 2, "protocol_version": 3,
                "execution": "corto_direct", "termination_policy": "goal_or_planner_end_or_timeout",
                "generation_seed": args.seed,
                "start": START, "goal": GOAL, "initial_yaw_rad": math.pi,
                "initial_quaternion_wxyz": [0, 0, 0, 1], "goal_tolerance_m": 0.2,
                "collision_model": "sampled_center_point",
                "flight_timeout_s": 60.0, "wall_timeout_s": 180.0,
                "v_max_mps": 10, "a_max_mps2": 5, "nmpc_horizon": 10,
                "nmpc_dt_s": 0.05, "socp_nodes": 100, "scenes": manifests,
                "runs": [{"run_id": f"direct_{name}_{seed:02d}", "scene": name, "planner_seed": seed}
                         for name in COUNTS for seed in range(10)]}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
