#!/usr/bin/env python3
"""Test one assembled GO4 RL joint around a captured safe reference pose."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_leg_adapter import (
    RL_MOTOR_ORDER,
    joint_delta_to_motor_output_delta_deg,
    motor_output_delta_to_joint_delta_deg,
)
from go4_rl_kinematics import Go4RLKinematics, RL_JOINT_NAMES
from unitree_daisy_chain import (
    UnitreeDaisyChain,
    import_unitree_sdk,
    unwrap_near,
)


SCHEMA = "go4_rl_relative_reference_v1"
DEFAULT_REFERENCE_FILE = os.path.expanduser("~/go4_rl_reference.json")
DEFAULT_URDF = Path(__file__).resolve().parents[1] / "models" / "go4" / "GO4.urdf"
ROLE_TO_URDF_JOINT = dict(zip(RL_MOTOR_ORDER, RL_JOINT_NAMES))


def load_reference(path):
    with open(Path(path).expanduser(), "r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if payload.get("schema") != SCHEMA:
        raise ValueError(
            f"reference schema={payload.get('schema')!r}, expected {SCHEMA!r}"
        )
    motors = payload.get("motors")
    if not isinstance(motors, dict):
        raise ValueError("reference has no motors mapping")
    ids = []
    for role in RL_MOTOR_ORDER:
        entry = motors.get(role)
        if not isinstance(entry, dict):
            raise ValueError(f"reference has no {role} entry")
        motor_id = int(entry["id"])
        q_reference = float(entry["q_reference_phase_rad"])
        if motor_id < 0 or motor_id > 14 or not math.isfinite(q_reference):
            raise ValueError(f"invalid {role} reference entry")
        ids.append(motor_id)
    if len(set(ids)) != len(ids):
        raise ValueError("reference contains duplicate motor IDs")
    gear = float(payload.get("internal_gear_ratio", float("nan")))
    if not math.isfinite(gear) or gear <= 0.0:
        raise ValueError("reference contains invalid internal gear ratio")
    transmission = payload.get("knee_external_transmission", {})
    if (
        int(transmission.get("motor_teeth", 0)) != 16
        or int(transmission.get("knee_teeth", 0)) != 28
        or float(transmission.get("parallelogram_ratio", 0.0)) != 1.0
    ):
        raise ValueError("reference does not declare the expected 16:28, 1:1 knee drive")
    return payload


def safe_test_sequence(reference, role, step_deg, urdf_path):
    """Choose only directions that remain inside the GO4 URDF joint limits."""

    urdf = reference.get("urdf")
    joint_reference = urdf.get("joint_reference_deg") if isinstance(urdf, dict) else None
    joint_name = ROLE_TO_URDF_JOINT[role]
    if not isinstance(joint_reference, dict) or joint_name not in joint_reference:
        raise ValueError(
            "reference has no GO4 URDF joint reference; recapture it with script 43"
        )

    model = Go4RLKinematics.from_urdf(urdf_path)
    joint_index = RL_JOINT_NAMES.index(joint_name)
    reference_deg = float(joint_reference[joint_name])
    lower_deg = math.degrees(model.lower[joint_index])
    upper_deg = math.degrees(model.upper[joint_index])
    tolerance = 1e-4
    if reference_deg < lower_deg - tolerance or reference_deg > upper_deg + tolerance:
        raise ValueError(
            f"reference {joint_name}={reference_deg:+.3f} deg is outside "
            f"URDF [{lower_deg:+.3f}, {upper_deg:+.3f}] deg"
        )

    allowed = []
    if reference_deg + step_deg <= upper_deg + tolerance:
        allowed.append(step_deg)
    if reference_deg - step_deg >= lower_deg - tolerance:
        allowed.append(-step_deg)
    if not allowed:
        raise ValueError(
            f"no {step_deg:.3f} deg test step fits inside the URDF limits"
        )
    sequence = [0.0]
    for delta in allowed:
        sequence.extend((delta, 0.0))
    return tuple(sequence), reference_deg, lower_deg, upper_deg


def cosine_blend(ratio):
    ratio = min(max(float(ratio), 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * ratio)


def read_stable(bus, motor_id, settle_timeout_sec):
    deadline = time.monotonic() + settle_timeout_sec
    while True:
        try:
            return bus.read_mean_q(motor_id, samples=10)
        except RuntimeError as exc:
            if "unstable rotor readings" not in str(exc) or time.monotonic() >= deadline:
                raise
            print(f"waiting for motor to settle: {exc}")
            time.sleep(0.25)


def joint_delta_from_rotor(q, q_reference, gear, role, direction):
    motor_output_delta = math.degrees(q - q_reference) / gear
    return motor_output_delta_to_joint_delta_deg(
        role, motor_output_delta, direction
    )


def move_segment(bus, motor_id, q_start, q_target, q_reference, gear, args):
    started = time.monotonic()
    deadline = started + args.ramp_sec + args.convergence_timeout_sec
    last_q = q_start
    excessive_error_count = 0
    converged_count = 0
    target_joint_delta = joint_delta_from_rotor(
        q_target, q_reference, gear, args.joint, args.direction
    )
    max_lead_motor_deg = abs(
        joint_delta_to_motor_output_delta_deg(
            args.joint, args.max_command_lead_deg, args.direction
        )
    )
    max_lead_rotor_rad = math.radians(max_lead_motor_deg) * gear
    while True:
        now = time.monotonic()
        elapsed = now - started
        ratio = min(elapsed / args.ramp_sec, 1.0)
        q_nominal = q_start + (q_target - q_start) * cosine_blend(ratio)
        q_cmd = min(
            max(q_nominal, last_q - max_lead_rotor_rad),
            last_q + max_lead_rotor_rad,
        )
        reply = bus.transact(
            motor_id,
            q=q_cmd,
            dq=0.0,
            kp=args.kp,
            kd=args.kd,
            tau=0.0,
        )
        last_q = unwrap_near(reply.q, q_cmd)
        commanded_joint_delta = joint_delta_from_rotor(
            q_cmd, q_reference, gear, args.joint, args.direction
        )
        actual_joint_delta = joint_delta_from_rotor(
            last_q, q_reference, gear, args.joint, args.direction
        )
        tracking_error = commanded_joint_delta - actual_joint_delta
        target_error = target_joint_delta - actual_joint_delta
        max_excursion = max(
            args.start_tolerance_deg,
            args.step_deg + args.max_excursion_margin_deg,
        )
        if abs(actual_joint_delta) > max_excursion:
            raise RuntimeError(
                f"joint left relative safety envelope: actual={actual_joint_delta:+.2f} "
                f"deg, limit=+/-{max_excursion:.2f} deg"
            )
        if abs(tracking_error) > args.max_tracking_error_deg:
            excessive_error_count += 1
        else:
            excessive_error_count = 0
        if excessive_error_count >= 5:
            raise RuntimeError(
                f"joint tracking error stayed above limit: {tracking_error:+.2f} deg"
            )
        if ratio >= 1.0 and abs(target_error) <= args.position_tolerance_deg:
            converged_count += 1
        else:
            converged_count = 0
        if converged_count >= 5:
            print(
                f"segment done: target={target_joint_delta:+.2f} deg, "
                f"actual={actual_joint_delta:+.2f} deg, "
                f"error={target_error:+.2f} deg"
            )
            break
        if now >= deadline:
            raise RuntimeError(
                f"joint did not converge within {args.convergence_timeout_sec:.1f}s "
                f"after ramp: target_error={target_error:+.2f} deg"
            )
        time.sleep(args.dt)
    return last_q


def hold(bus, motor_id, q_target, q_reference, gear, args):
    until = time.monotonic() + args.hold_sec
    last_q = q_target
    while time.monotonic() < until:
        reply = bus.transact(
            motor_id,
            q=q_target,
            dq=0.0,
            kp=args.kp,
            kd=args.kd,
            tau=0.0,
        )
        last_q = unwrap_near(reply.q, q_target)
        target_joint_delta = joint_delta_from_rotor(
            q_target, q_reference, gear, args.joint, args.direction
        )
        actual_joint_delta = joint_delta_from_rotor(
            last_q, q_reference, gear, args.joint, args.direction
        )
        if abs(target_joint_delta - actual_joint_delta) > args.max_tracking_error_deg:
            raise RuntimeError(
                "hold joint tracking error exceeds limit: "
                f"{target_joint_delta - actual_joint_delta:+.2f} deg"
            )
        time.sleep(args.dt)
    return last_q


def fade_stiffness(bus, motor_id, q_hold, args):
    for index in range(30):
        scale = 1.0 - index / 30.0
        bus.transact(
            motor_id,
            q=q_hold,
            dq=0.0,
            kp=args.kp * scale,
            kd=args.kd * scale,
            tau=0.0,
            allow_fault=True,
        )
        time.sleep(args.dt)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--reference", default=DEFAULT_REFERENCE_FILE)
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--joint", choices=RL_MOTOR_ORDER, required=True)
    parser.add_argument("--direction", type=float, choices=(-1.0, 1.0), required=True)
    parser.add_argument("--step-deg", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=0.10)
    parser.add_argument("--kd", type=float, default=0.04)
    parser.add_argument("--ramp-sec", type=float, default=5.0)
    parser.add_argument("--hold-sec", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--settle-timeout-sec", type=float, default=5.0)
    parser.add_argument("--convergence-timeout-sec", type=float, default=5.0)
    parser.add_argument("--max-command-lead-deg", type=float, default=0.75)
    parser.add_argument("--position-tolerance-deg", type=float, default=0.40)
    parser.add_argument("--max-excursion-margin-deg", type=float, default=1.5)
    parser.add_argument("--start-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=1.5)
    parser.add_argument("--transmission-installed", action="store_true")
    parser.add_argument("--joint-supported", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()

    numeric = (
        args.step_deg,
        args.kp,
        args.kd,
        args.ramp_sec,
        args.hold_sec,
        args.dt,
        args.settle_timeout_sec,
        args.convergence_timeout_sec,
        args.max_command_lead_deg,
        args.position_tolerance_deg,
        args.max_excursion_margin_deg,
        args.start_tolerance_deg,
        args.max_tracking_error_deg,
    )
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if not 0.0 < args.step_deg <= 2.0:
        parser.error("assembled joint step-deg must be in (0, 2]")
    if args.kp < 0.0 or args.kd < 0.0:
        parser.error("kp and kd must be non-negative")
    if min(
        args.ramp_sec,
        args.dt,
        args.settle_timeout_sec,
        args.convergence_timeout_sec,
        args.max_command_lead_deg,
        args.position_tolerance_deg,
        args.max_excursion_margin_deg,
        args.start_tolerance_deg,
        args.max_tracking_error_deg,
    ) <= 0.0 or args.hold_sec < 0.0:
        parser.error("timing and tolerance values must be positive")

    try:
        reference = load_reference(args.reference)
    except Exception as exc:
        print(f"Invalid reference file: {exc}")
        return 1
    entry = reference["motors"][args.joint]
    motor_id = int(entry["id"])
    bus_ids = [int(reference["motors"][role]["id"]) for role in RL_MOTOR_ORDER]
    try:
        sequence, reference_joint_deg, lower_deg, upper_deg = safe_test_sequence(
            reference, args.joint, args.step_deg, args.urdf
        )
    except Exception as exc:
        print(f"Cannot build URDF-safe test sequence: {exc}")
        return 1
    first_step = next(delta for delta in sequence if delta != 0.0)
    motor_output_step = joint_delta_to_motor_output_delta_deg(
        args.joint, first_step, args.direction
    )

    print("GO4 RL assembled joint relative test")
    print(f"joint={args.joint} id={motor_id} direction={args.direction:+.0f}")
    print(f"URDF reference={reference_joint_deg:+.3f} deg")
    print(f"URDF limits=[{lower_deg:+.3f}, {upper_deg:+.3f}] deg")
    print("joint delta sequence=" + " -> ".join(f"{value:+.2f}" for value in sequence))
    print(f"required motor-output step={motor_output_step:+.3f} deg")
    print("This is a RELATIVE test; the captured pose is not URDF joint zero.")

    if not (
        args.transmission_installed and args.joint_supported and args.enable_motion
    ):
        print("DRY RUN only. Motion requires all three safety flags:")
        print("  --transmission-installed --joint-supported --enable-motion")
        return 0

    sdk = import_unitree_sdk(args.sdk_path)
    port = str(reference["port"])
    bus = UnitreeDaisyChain(sdk, port)
    gear = bus.gear_ratio()
    saved_gear = float(reference["internal_gear_ratio"])
    if abs(gear - saved_gear) > 1e-6:
        print(f"Gear ratio mismatch: SDK={gear}, reference={saved_gear}")
        return 1

    q_last = None
    result = 0
    try:
        bus.stop_many(bus_ids, repeats=3)
        current_by_role = {}
        aligned_reference_by_role = {}
        for role in RL_MOTOR_ORDER:
            role_entry = reference["motors"][role]
            role_id = int(role_entry["id"])
            role_current = read_stable(
                bus, role_id, args.settle_timeout_sec
            )
            role_reference = unwrap_near(
                float(role_entry["q_reference_phase_rad"]), role_current
            )
            # Direction changes only the sign; proximity validation needs magnitude.
            role_offset = joint_delta_from_rotor(
                role_current, role_reference, gear, role, 1.0
            )
            print(f"{role} offset from captured pose={role_offset:+.2f} joint deg")
            if abs(role_offset) > args.start_tolerance_deg:
                raise RuntimeError(
                    f"{role} differs from reference by {role_offset:+.2f} deg; "
                    f"limit is {args.start_tolerance_deg:.2f} deg"
                )
            current_by_role[role] = role_current
            aligned_reference_by_role[role] = role_reference
            bus.stop(role_id)

        q_current = current_by_role[args.joint]
        q_reference = aligned_reference_by_role[args.joint]

        print("\nBefore continuing:")
        print("  1. The leg must be supported against gravity.")
        print("  2. The two non-tested joints must be mechanically restrained.")
        print("  3. Keep clear of gears, links, hard stops and pinch points.")
        answer = input("Type MOVE to run this one assembled joint: ").strip().upper()
        if answer != "MOVE":
            print("Cancelled.")
        else:
            q_last = q_current
            for target_joint_delta in sequence:
                motor_delta = joint_delta_to_motor_output_delta_deg(
                    args.joint, target_joint_delta, args.direction
                )
                q_target = q_reference + math.radians(motor_delta) * gear
                q_last = move_segment(
                    bus,
                    motor_id,
                    q_last,
                    q_target,
                    q_reference,
                    gear,
                    args,
                )
                q_last = hold(
                    bus, motor_id, q_target, q_reference, gear, args
                )
                actual_joint_delta = joint_delta_from_rotor(
                    q_last, q_reference, gear, args.joint, args.direction
                )
                print(
                    f"hold done: target={target_joint_delta:+.2f} deg, "
                    f"actual={actual_joint_delta:+.2f} deg, "
                    f"error={target_joint_delta - actual_joint_delta:+.2f} deg"
                )
    except KeyboardInterrupt:
        result = 130
        print("Interrupted: stopping all bus motors.")
    except Exception as exc:
        result = 1
        print(f"FAULT: {exc}")
    finally:
        if q_last is not None:
            try:
                fade_stiffness(bus, motor_id, q_last, args)
            except Exception as exc:
                result = 1
                print(f"stiffness fade failed: {exc}; still sending STOP")
        try:
            bus.stop_many(bus_ids, repeats=5)
            print("Stop replies confirmed for all configured bus IDs.")
        except Exception as exc:
            result = 1
            print(f"STOP FAILED: {exc}; cut motor power immediately.")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
