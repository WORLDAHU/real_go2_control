#!/usr/bin/env python3
"""Run the scaled legacy GO4 motion as an ungeared three-motor equivalent."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from go4_leg_adapter import RL_MOTOR_ORDER, joint_delta_to_motor_output_delta_deg
from go4_rl_kinematics import Go4RLKinematics
from unitree_daisy_chain import UnitreeDaisyChain, import_unitree_sdk, unwrap_near


URDF = ROOT / "models" / "go4" / "GO4.urdf"
REFERENCE = os.path.expanduser("~/go4_rl_reference.json")
DIRECTIONS = {"hip": -1.0, "thigh": 1.0, "knee": -1.0}


def min_jerk(value):
    value = min(max(float(value), 0.0), 1.0)
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", default=REFERENCE)
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--stand-mm", type=float, default=30.0)
    parser.add_argument("--crouch-mm", type=float, default=0.0)
    parser.add_argument("--extend-mm", type=float, default=60.0)
    parser.add_argument("--soft-land-mm", type=float, default=10.0)
    parser.add_argument("--segment-sec", type=float, default=8.0)
    parser.add_argument("--flight-hold-sec", type=float, default=1.0)
    parser.add_argument("--prehold-sec", type=float, default=3.0)
    parser.add_argument("--max-speed-deg-s", type=float, default=1.5)
    parser.add_argument("--velocity-feedforward-scale", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=0.30)
    parser.add_argument("--kd", type=float, default=0.06)
    parser.add_argument("--hip-kp", type=float, default=0.40)
    parser.add_argument("--hip-kd", type=float, default=0.08)
    parser.add_argument("--knee-kp", type=float, default=0.35)
    parser.add_argument("--knee-kd", type=float, default=0.07)
    parser.add_argument("--max-start-offset-deg", type=float, default=15.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=4.0)
    parser.add_argument("--knee-max-tracking-error-deg", type=float, default=5.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--gears-not-installed", action="store_true")
    parser.add_argument("--shafts-free", action="store_true")
    parser.add_argument("--bare-reference-confirmed", action="store_true")
    parser.add_argument("--large-range-confirmed", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()

    depths = (args.stand_mm, args.crouch_mm, args.extend_mm, args.soft_land_mm)
    if not all(math.isfinite(value) for value in depths):
        parser.error("all depths must be finite")
    if not 0.0 <= args.crouch_mm <= args.soft_land_mm <= args.stand_mm < args.extend_mm:
        parser.error("expected crouch <= soft-land <= stand < extend")
    if args.extend_mm > 160.0:
        parser.error("bare replicated motion is limited to 160 mm")
    positive = (
        args.segment_sec,
        args.prehold_sec,
        args.max_speed_deg_s,
        args.max_start_offset_deg,
        args.max_tracking_error_deg,
        args.knee_max_tracking_error_deg,
        args.dt,
    )
    nonnegative = (
        args.flight_hold_sec,
        args.kp,
        args.kd,
        args.hip_kp,
        args.hip_kd,
        args.knee_kp,
        args.knee_kd,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        parser.error("timing, speed and tolerances must be positive and finite")
    if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
        parser.error("hold time and gains must be non-negative and finite")
    if not math.isfinite(args.velocity_feedforward_scale) or not 0.0 <= args.velocity_feedforward_scale <= 1.0:
        parser.error("velocity-feedforward-scale must be in [0, 1]")

    model = Go4RLKinematics.from_urdf(URDF)
    q_reference = model.retracted_q()
    phase_names = ("home", "stand", "crouch", "push", "absorb", "recover", "home")
    phase_depths_mm = np.asarray(
        [0.0, args.stand_mm, args.crouch_mm, args.extend_mm, args.soft_land_mm, args.stand_mm, 0.0]
    )
    phase_q = model.extension_keyframes(phase_depths_mm / 1000.0)

    def output_for_q(q):
        joint_delta = np.degrees(q - q_reference)
        return {
            role: joint_delta_to_motor_output_delta_deg(role, joint_delta[index], DIRECTIONS[role])
            for index, role in enumerate(RL_MOTOR_ORDER)
        }

    phase_output = [output_for_q(q) for q in phase_q]
    kp = {"hip": args.hip_kp, "thigh": args.kp, "knee": args.knee_kp}
    kd = {"hip": args.hip_kd, "thigh": args.kd, "knee": args.knee_kd}
    max_error = {
        "hip": args.max_tracking_error_deg,
        "thigh": args.max_tracking_error_deg,
        "knee": args.knee_max_tracking_error_deg,
    }

    print("GO4 RL bare-motor scaled legacy-motion demo")
    print("motion: home -> stand -> crouch -> push -> flight -> absorb -> recover -> home")
    for name, depth, output in zip(phase_names, phase_depths_mm, phase_output):
        print(
            f"{name:>7s}: depth={depth:5.1f} mm, "
            + ", ".join(f"{role}={output[role]:+.3f} output deg" for role in RL_MOTOR_ORDER)
        )
    print(
        "gains: "
        + ", ".join(f"{role}={kp[role]:.3f}/{kd[role]:.3f}" for role in RL_MOTOR_ORDER)
    )
    print(f"velocity feedforward scale={args.velocity_feedforward_scale:.2f}")
    required_flags = (
        args.gears_not_installed
        and args.shafts_free
        and args.bare_reference_confirmed
        and args.enable_motion
    )
    large_range_ready = args.extend_mm <= 60.0 or args.large_range_confirmed
    if not (required_flags and large_range_ready):
        print("DRY RUN. Motion requires --gears-not-installed --shafts-free")
        print("--bare-reference-confirmed --enable-motion")
        if args.extend_mm > 60.0:
            print("Motion above 60 mm also requires --large-range-confirmed")
        return 0

    reference = json.loads(Path(args.reference).expanduser().read_text(encoding="utf-8"))
    if reference.get("schema") != "go4_rl_relative_reference_v1":
        raise SystemExit("invalid reference; capture the bare zero with script 43")
    motors = reference.get("motors", {})
    if any(role not in motors for role in RL_MOTOR_ORDER):
        raise SystemExit("reference is missing a GO4 motor role")
    ids = [int(motors[role]["id"]) for role in RL_MOTOR_ORDER]
    if len(set(ids)) != 3:
        raise SystemExit("reference motor IDs must be unique")
    bus = UnitreeDaisyChain(import_unitree_sdk(args.sdk_path), reference["port"])
    gear = bus.gear_ratio()
    if abs(gear - float(reference.get("internal_gear_ratio", float("nan")))) > 1e-6:
        raise SystemExit("SDK internal gear ratio differs from reference")

    zero = {}
    current = {}
    last = {}
    excessive = {role: 0 for role in RL_MOTOR_ORDER}
    commanded_peak = {role: 0.0 for role in RL_MOTOR_ORDER}
    measured_peak = {role: 0.0 for role in RL_MOTOR_ORDER}
    result = 0

    def transact_targets(targets, velocities=None):
        velocities = velocities or {role: 0.0 for role in RL_MOTOR_ORDER}
        for role in RL_MOTOR_ORDER:
            motor_id = int(motors[role]["id"])
            reply = bus.transact(
                motor_id,
                q=targets[role],
                dq=velocities[role],
                kp=kp[role],
                kd=kd[role],
                tau=0.0,
            )
            last[role] = unwrap_near(reply.q, targets[role])
            commanded_speed = abs(math.degrees(velocities[role]) / gear)
            commanded_peak[role] = max(commanded_peak[role], commanded_speed)
            if math.isfinite(reply.dq):
                measured_speed = abs(math.degrees(reply.dq) / gear)
                measured_peak[role] = max(measured_peak[role], measured_speed)
            error = abs(math.degrees(last[role] - targets[role]) / gear)
            excessive[role] = excessive[role] + 1 if error > max_error[role] else 0
            if excessive[role] >= 5:
                raise RuntimeError(f"{role} tracking error={error:.2f} output deg")

    def move_targets(name, starts, targets):
        distance = max(abs(math.degrees(targets[r] - starts[r]) / gear) for r in RL_MOTOR_ORDER)
        duration = max(args.segment_sec, 2.1 * distance / args.max_speed_deg_s)
        print(f"phase={name}: duration={duration:.2f}s")
        begun = time.monotonic()
        while True:
            ratio = min((time.monotonic() - begun) / duration, 1.0)
            blend = min_jerk(ratio)
            blend_rate = 30.0 * ratio**2 * (1.0 - ratio) ** 2 / duration if ratio < 1.0 else 0.0
            commands = {r: starts[r] + (targets[r] - starts[r]) * blend for r in RL_MOTOR_ORDER}
            velocities = {
                r: (targets[r] - starts[r]) * blend_rate * args.velocity_feedforward_scale
                for r in RL_MOTOR_ORDER
            }
            transact_targets(commands, velocities)
            if ratio >= 1.0:
                break
            time.sleep(args.dt)
        return dict(last)

    def hold_targets(name, targets, duration):
        if duration <= 0.0:
            return
        print(f"phase={name}: holding {duration:.2f}s")
        until = time.monotonic() + duration
        while time.monotonic() < until:
            transact_targets(targets)
            time.sleep(args.dt)

    try:
        print("Support the three free shafts, then confirm.")
        answer = input("Type BARE_LEGACY to acquire and run: ").strip().upper()
        if answer != "BARE_LEGACY":
            print("Cancelled")
            return 0

        bus.stop_many(ids, repeats=3)
        for role in RL_MOTOR_ORDER:
            current[role] = bus.query(int(motors[role]["id"])).q
        for _ in range(12):
            for role in RL_MOTOR_ORDER:
                motor_id = int(motors[role]["id"])
                last[role] = unwrap_near(
                    bus.transact(motor_id, q=current[role], dq=0.0, kp=kp[role], kd=kd[role], tau=0.0).q,
                    current[role],
                )
            time.sleep(args.dt)
        current = dict(last)
        for role in RL_MOTOR_ORDER:
            zero[role] = unwrap_near(float(motors[role]["q_reference_phase_rad"]), current[role])
            offset = math.degrees(current[role] - zero[role]) / gear
            print(f"{role}: start offset={offset:+.3f} output deg")
            if abs(offset) > args.max_start_offset_deg:
                raise RuntimeError(f"{role} is not close enough to bare reference")

        targets = []
        for output in phase_output:
            targets.append(
                {
                    role: zero[role] + math.radians(output[role]) * gear
                    for role in RL_MOTOR_ORDER
                }
            )

        current = move_targets("home", current, targets[0])
        hold_targets("home_hold", targets[0], args.prehold_sec)
        print("All motors at zero. You may release support.")
        for index in range(1, len(targets)):
            current = move_targets(phase_names[index], current, targets[index])
            if phase_names[index] == "push":
                hold_targets("flight", targets[index], args.flight_hold_sec)
        hold_targets("final_home_hold", targets[-1], 1.0)
        print("Bare-motor scaled legacy motion completed and returned to zero.")
        print("peak output speed [deg/s]:")
        for role in RL_MOTOR_ORDER:
            print(
                f"  {role}: commanded={commanded_peak[role]:.2f}, "
                f"measured={measured_peak[role]:.2f}"
            )
    except KeyboardInterrupt:
        result = 130
        print("Interrupted")
    except Exception as exc:
        result = 1
        print(f"FAULT: {exc}")
    finally:
        try:
            bus.stop_many(ids, repeats=5)
            print("Stop replies confirmed for all IDs.")
        except Exception as exc:
            result = 1
            print(f"STOP FAILED: {exc}; cut power")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
