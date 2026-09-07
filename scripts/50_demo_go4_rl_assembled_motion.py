#!/usr/bin/env python3
"""Run a slow, scaled replica of the legacy single-leg cycle on assembled GO4 RL."""

import argparse
import hashlib
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


def load_reference(path, urdf_path):
    payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if payload.get("schema") != "go4_rl_relative_reference_v1":
        raise ValueError("invalid GO4 reference schema; run script 43 after assembly")
    motors = payload.get("motors")
    if not isinstance(motors, dict):
        raise ValueError("reference has no motors mapping")
    ids = []
    for role in RL_MOTOR_ORDER:
        entry = motors.get(role)
        if not isinstance(entry, dict):
            raise ValueError(f"reference has no {role} entry")
        motor_id = int(entry["id"])
        phase = float(entry["q_reference_phase_rad"])
        if not 0 <= motor_id <= 14 or not math.isfinite(phase):
            raise ValueError(f"invalid {role} reference entry")
        ids.append(motor_id)
    if len(set(ids)) != 3:
        raise ValueError("reference motor IDs must be unique")
    transmission = payload.get("knee_external_transmission", {})
    if (
        int(transmission.get("motor_teeth", 0)) != 16
        or int(transmission.get("knee_teeth", 0)) != 28
        or float(transmission.get("parallelogram_ratio", 0.0)) != 1.0
    ):
        raise ValueError("reference does not declare GO4 16:28 and 1:1 knee transmission")
    expected_hash = hashlib.sha256(Path(urdf_path).read_bytes()).hexdigest()
    if payload.get("urdf", {}).get("sha256") != expected_hash:
        raise ValueError("reference URDF hash differs; recapture with script 43")
    return payload


def validate_profile(parser, args):
    depths = (args.stand_mm, args.crouch_mm, args.extend_mm, args.soft_land_mm)
    if not all(math.isfinite(value) for value in depths):
        parser.error("all motion depths must be finite")
    if not 0.0 <= args.crouch_mm <= args.soft_land_mm <= args.stand_mm < args.extend_mm:
        parser.error("expected crouch <= soft-land <= stand < extend")
    if args.extend_mm > 60.0:
        parser.error("first assembled replica is limited to 60 mm")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", default=REFERENCE)
    parser.add_argument("--urdf", default=str(URDF))
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--stand-mm", type=float, default=30.0)
    parser.add_argument("--crouch-mm", type=float, default=0.0)
    parser.add_argument("--extend-mm", type=float, default=60.0)
    parser.add_argument("--soft-land-mm", type=float, default=10.0)
    parser.add_argument("--home-sec", type=float, default=5.0)
    parser.add_argument("--stand-sec", type=float, default=5.0)
    parser.add_argument("--crouch-sec", type=float, default=5.0)
    parser.add_argument("--push-sec", type=float, default=8.0)
    parser.add_argument("--flight-hold-sec", type=float, default=1.0)
    parser.add_argument("--absorb-sec", type=float, default=8.0)
    parser.add_argument("--recover-sec", type=float, default=5.0)
    parser.add_argument("--final-home-sec", type=float, default=5.0)
    parser.add_argument("--prehold-sec", type=float, default=3.0)
    parser.add_argument("--max-speed-deg-s", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=0.20)
    parser.add_argument("--kd", type=float, default=0.04)
    parser.add_argument("--hip-kp", type=float, default=0.40)
    parser.add_argument("--hip-kd", type=float, default=0.08)
    parser.add_argument("--knee-kp", type=float, default=0.35)
    parser.add_argument("--knee-kd", type=float, default=0.07)
    parser.add_argument("--max-start-offset-deg", type=float, default=3.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=3.0)
    parser.add_argument("--hip-max-tracking-error-deg", type=float, default=3.0)
    parser.add_argument("--knee-max-tracking-error-deg", type=float, default=4.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--transmission-installed", action="store_true")
    parser.add_argument("--leg-suspended", action="store_true")
    parser.add_argument("--assembled-reference-confirmed", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()
    validate_profile(parser, args)
    positive = (
        args.home_sec,
        args.stand_sec,
        args.crouch_sec,
        args.push_sec,
        args.absorb_sec,
        args.recover_sec,
        args.final_home_sec,
        args.prehold_sec,
        args.max_speed_deg_s,
        args.max_start_offset_deg,
        args.max_tracking_error_deg,
        args.hip_max_tracking_error_deg,
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

    model = Go4RLKinematics.from_urdf(args.urdf)
    q_reference = model.retracted_q()
    key_names = ("home", "stand", "crouch", "push", "absorb", "recover", "home")
    key_depths_mm = np.asarray(
        [0.0, args.stand_mm, args.crouch_mm, args.extend_mm, args.soft_land_mm, args.stand_mm, 0.0]
    )
    key_q = model.extension_keyframes(key_depths_mm / 1000.0)

    kp = {"hip": args.hip_kp, "thigh": args.kp, "knee": args.knee_kp}
    kd = {"hip": args.hip_kd, "thigh": args.kd, "knee": args.knee_kd}
    max_error = {
        "hip": args.hip_max_tracking_error_deg,
        "thigh": args.max_tracking_error_deg,
        "knee": args.knee_max_tracking_error_deg,
    }

    def motor_output_for_q(q):
        joint_delta = np.degrees(q - q_reference)
        return {
            role: joint_delta_to_motor_output_delta_deg(role, joint_delta[index], DIRECTIONS[role])
            for index, role in enumerate(RL_MOTOR_ORDER)
        }

    key_output = [motor_output_for_q(q) for q in key_q]
    print("GO4 RL assembled scaled legacy-motion demo")
    print("motion: home -> stand -> crouch -> push -> flight -> absorb -> recover -> home")
    for name, depth, output in zip(key_names, key_depths_mm, key_output):
        print(
            f"{name:>7s}: depth={depth:5.1f} mm, motor output "
            + ", ".join(f"{role}={output[role]:+.3f} deg" for role in RL_MOTOR_ORDER)
        )
    print(
        "gains: "
        + ", ".join(f"{role}={kp[role]:.3f}/{kd[role]:.3f}" for role in RL_MOTOR_ORDER)
    )
    required_flags = (
        args.transmission_installed,
        args.leg_suspended,
        args.assembled_reference_confirmed,
        args.enable_motion,
    )
    if not all(required_flags):
        print("DRY RUN. Motion requires --transmission-installed --leg-suspended")
        print("--assembled-reference-confirmed --enable-motion")
        return 0

    reference = load_reference(args.reference, args.urdf)
    motors = reference["motors"]
    ids = [int(motors[role]["id"]) for role in RL_MOTOR_ORDER]
    bus = UnitreeDaisyChain(import_unitree_sdk(args.sdk_path), reference["port"])
    gear = bus.gear_ratio()
    if abs(gear - float(reference["internal_gear_ratio"])) > 1e-6:
        raise SystemExit("SDK internal gear ratio differs from captured reference")

    grid_depths = np.linspace(0.0, args.extend_mm / 1000.0, 241)
    grid_q = model.extension_keyframes(grid_depths)

    def q_for_depth(depth_m):
        return np.asarray(
            [np.interp(depth_m, grid_depths, grid_q[:, index]) for index in range(3)]
        )

    zero = {}
    current = {}
    last = {}
    excessive = {role: 0 for role in RL_MOTOR_ORDER}
    result = 0

    def rotor_targets_for_depth(depth_m):
        output = motor_output_for_q(q_for_depth(depth_m))
        return {
            role: zero[role] + math.radians(output[role]) * gear
            for role in RL_MOTOR_ORDER
        }

    def transact_targets(targets):
        for role in RL_MOTOR_ORDER:
            motor_id = int(motors[role]["id"])
            last[role] = unwrap_near(
                bus.transact(
                    motor_id,
                    q=targets[role],
                    dq=0.0,
                    kp=kp[role],
                    kd=kd[role],
                    tau=0.0,
                ).q,
                targets[role],
            )
            error = abs(math.degrees(last[role] - targets[role]) / gear)
            excessive[role] = excessive[role] + 1 if error > max_error[role] else 0
            if excessive[role] >= 5:
                raise RuntimeError(f"{role} tracking error={error:.2f} output deg")

    def move_targets(name, starts, targets, requested_sec):
        distance = max(abs(math.degrees(targets[r] - starts[r]) / gear) for r in RL_MOTOR_ORDER)
        duration = max(requested_sec, 2.1 * distance / args.max_speed_deg_s)
        print(f"phase={name}: duration={duration:.2f}s")
        begun = time.monotonic()
        while True:
            ratio = min((time.monotonic() - begun) / duration, 1.0)
            blend = min_jerk(ratio)
            transact_targets({r: starts[r] + (targets[r] - starts[r]) * blend for r in RL_MOTOR_ORDER})
            if ratio >= 1.0:
                break
            time.sleep(args.dt)
        return dict(last)

    def move_depth(name, start_depth_m, target_depth_m, requested_sec):
        start_targets = rotor_targets_for_depth(start_depth_m)
        end_targets = rotor_targets_for_depth(target_depth_m)
        distance = max(
            abs(math.degrees(end_targets[r] - start_targets[r]) / gear)
            for r in RL_MOTOR_ORDER
        )
        duration = max(requested_sec, 2.1 * distance / args.max_speed_deg_s)
        print(f"phase={name}: depth {start_depth_m*1000:.1f}->{target_depth_m*1000:.1f} mm, duration={duration:.2f}s")
        begun = time.monotonic()
        while True:
            ratio = min((time.monotonic() - begun) / duration, 1.0)
            blend = min_jerk(ratio)
            depth = start_depth_m + (target_depth_m - start_depth_m) * blend
            transact_targets(rotor_targets_for_depth(depth))
            if ratio >= 1.0:
                break
            time.sleep(args.dt)

    def hold_depth(name, depth_m, duration):
        if duration <= 0.0:
            return
        print(f"phase={name}: holding {duration:.2f}s")
        targets = rotor_targets_for_depth(depth_m)
        until = time.monotonic() + duration
        while time.monotonic() < until:
            transact_targets(targets)
            time.sleep(args.dt)

    try:
        print("Support and restrain the assembled leg, then confirm.")
        answer = input("Type ASSEMBLED_DEMO to acquire and run: ").strip().upper()
        if answer != "ASSEMBLED_DEMO":
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
                raise RuntimeError(f"{role} is not close enough to assembled reference")

        current = move_targets("home", current, dict(zero), args.home_sec)
        hold_depth("home_hold", 0.0, args.prehold_sec)
        print("Home is held. Keep the leg suspended and clear of pinch points.")
        move_depth("stand", 0.0, args.stand_mm / 1000.0, args.stand_sec)
        move_depth("crouch", args.stand_mm / 1000.0, args.crouch_mm / 1000.0, args.crouch_sec)
        move_depth("push", args.crouch_mm / 1000.0, args.extend_mm / 1000.0, args.push_sec)
        hold_depth("flight", args.extend_mm / 1000.0, args.flight_hold_sec)
        move_depth("absorb", args.extend_mm / 1000.0, args.soft_land_mm / 1000.0, args.absorb_sec)
        move_depth("recover", args.soft_land_mm / 1000.0, args.stand_mm / 1000.0, args.recover_sec)
        move_depth("final_home", args.stand_mm / 1000.0, 0.0, args.final_home_sec)
        hold_depth("final_home_hold", 0.0, 1.0)
        print("Scaled legacy motion completed and returned to assembled reference.")
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
