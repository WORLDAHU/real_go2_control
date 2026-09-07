#!/usr/bin/env python3
"""Preview the scaled GO4 version of the legacy single-leg push cycle."""

import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from go4_leg_adapter import RL_MOTOR_ORDER, joint_delta_to_motor_output_delta_deg
from go4_rl_kinematics import Go4RLKinematics


URDF = ROOT / "models" / "go4" / "GO4.urdf"
DIRECTIONS = {"hip": -1.0, "thigh": 1.0, "knee": -1.0}
PHASES = (
    "home_start",
    "stand",
    "crouch",
    "push",
    "flight",
    "absorb",
    "recover",
    "home",
)


def min_jerk(value):
    value = min(max(float(value), 0.0), 1.0)
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def validate_depths(parser, args):
    values = (args.stand_mm, args.crouch_mm, args.extend_mm, args.soft_land_mm)
    if not all(math.isfinite(value) for value in values):
        parser.error("all depths must be finite")
    if not 0.0 <= args.crouch_mm <= args.soft_land_mm <= args.stand_mm < args.extend_mm:
        parser.error("expected crouch <= soft-land <= stand < extend")
    if args.extend_mm > 160.0:
        parser.error("preview extend depth must not exceed 160 mm")


def build_samples(model, key_depths_m, samples_per_move):
    depths = []
    phases = []
    for index, phase in enumerate(PHASES):
        if index == 0:
            depths.append(key_depths_m[0])
            phases.append(phase)
            continue
        start = key_depths_m[index - 1]
        target = key_depths_m[index]
        count = 2 if start == target else samples_per_move
        for sample in range(1, count + 1):
            depth = start + (target - start) * min_jerk(sample / count)
            depths.append(depth)
            phases.append(phase)
    return np.asarray(depths), phases, model.extension_keyframes(depths)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=str(URDF))
    parser.add_argument("--stand-mm", type=float, default=30.0)
    parser.add_argument("--crouch-mm", type=float, default=0.0)
    parser.add_argument("--extend-mm", type=float, default=60.0)
    parser.add_argument("--soft-land-mm", type=float, default=10.0)
    parser.add_argument("--samples-per-move", type=int, default=40)
    parser.add_argument("--csv")
    args = parser.parse_args()
    validate_depths(parser, args)
    if args.samples_per_move < 2:
        parser.error("samples-per-move must be at least 2")

    model = Go4RLKinematics.from_urdf(args.urdf)
    key_depths_mm = np.asarray(
        [
            0.0,
            args.stand_mm,
            args.crouch_mm,
            args.extend_mm,
            args.extend_mm,
            args.soft_land_mm,
            args.stand_mm,
            0.0,
        ]
    )
    key_q = model.extension_keyframes(key_depths_mm / 1000.0)
    q_reference = model.retracted_q()

    print("GO4 RL scaled legacy-motion preview")
    print(f"URDF: {Path(args.urdf).expanduser().resolve()}")
    print("motion: home -> stand -> crouch -> push -> flight -> absorb -> recover -> home")
    for phase, depth_mm, q in zip(PHASES, key_depths_mm, key_q):
        joint_delta_deg = np.degrees(q - q_reference)
        motor_delta_deg = [
            joint_delta_to_motor_output_delta_deg(role, joint_delta_deg[index], DIRECTIONS[role])
            for index, role in enumerate(RL_MOTOR_ORDER)
        ]
        print(
            f"{phase:>7s}: depth={depth_mm:6.1f} mm  "
            f"joint=[{np.degrees(q[0]):+8.3f}, {np.degrees(q[1]):+8.3f}, {np.degrees(q[2]):+8.3f}] deg  "
            f"motor-delta=[{motor_delta_deg[0]:+8.3f}, {motor_delta_deg[1]:+8.3f}, {motor_delta_deg[2]:+8.3f}] deg"
        )

    if args.csv:
        depths, phases, q_samples = build_samples(
            model, key_depths_mm / 1000.0, args.samples_per_move
        )
        output = Path(args.csv).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["phase", "depth_mm", "hip_joint_deg", "thigh_joint_deg", "knee_joint_deg"]
            )
            for phase, depth, q in zip(phases, depths, q_samples):
                writer.writerow([phase, depth * 1000.0, *np.degrees(q)])
        print(f"CSV: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
