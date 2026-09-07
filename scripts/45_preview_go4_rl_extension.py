#!/usr/bin/env python3
"""Offline preview of the GO4 RL retracted -> down -> retracted motion."""

import argparse
import csv
from pathlib import Path
import sys

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_rl_kinematics import Go4RLKinematics


DEFAULT_URDF = PROJECT_ROOT / "models" / "go4" / "GO4.urdf"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--stroke-mm", type=float, default=60.0)
    parser.add_argument("--duration-sec", type=float, default=4.0)
    parser.add_argument("--samples-per-leg", type=int, default=100)
    parser.add_argument("--csv", help="Optional output path for the preview samples")
    args = parser.parse_args()
    if args.duration_sec <= 0.0:
        parser.error("--duration-sec must be positive")

    model = Go4RLKinematics.from_urdf(args.urdf)
    trajectory = model.extension_cycle(
        stroke_m=args.stroke_mm / 1000.0,
        samples_per_leg=args.samples_per_leg,
    )
    foot = np.asarray([model.foot_position(q) for q in trajectory])
    time = np.linspace(0.0, args.duration_sec, len(trajectory))

    print("GO4 RL offline extension preview")
    print(f"URDF: {Path(args.urdf).resolve()}")
    print("motion: fully retracted -> foot down -> fully retracted")
    print(f"samples: {len(trajectory)}, duration: {args.duration_sec:.2f} s")
    print(f"foot start [m]: {foot[0]}")
    print(f"foot bottom [m]: {foot[len(foot) // 2]}")
    print(f"joint start [deg]: {np.degrees(trajectory[0])}")
    print(f"joint bottom [deg]: {np.degrees(trajectory[len(trajectory) // 2])}")
    print(f"joint min [deg]: {np.degrees(trajectory.min(axis=0))}")
    print(f"joint max [deg]: {np.degrees(trajectory.max(axis=0))}")

    if args.csv:
        output = Path(args.csv).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ("time_s", "hip_rad", "thigh_rad", "calf_rad", "foot_x_m", "foot_y_m", "foot_z_m")
            )
            for t, q, p in zip(time, trajectory, foot):
                writer.writerow((t, *q, *p))
        print(f"saved: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
