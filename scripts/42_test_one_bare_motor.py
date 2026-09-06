#!/usr/bin/env python3
"""Safely test one uncoupled GO4 motor around its current position."""

import argparse
import math
import sys
import time
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_leg_adapter import DEFAULT_RL_MOTOR_IDS, RL_MOTOR_ORDER
from unitree_daisy_chain import (
    UnitreeDaisyChain,
    import_unitree_sdk,
    unwrap_near,
)


def cosine_blend(ratio):
    ratio = min(max(float(ratio), 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * ratio)


def move_segment(bus, motor_id, q_start, q_target, gear, args):
    started = time.monotonic()
    last_q = q_start
    excessive_error_count = 0
    while True:
        elapsed = time.monotonic() - started
        ratio = min(elapsed / args.ramp_sec, 1.0)
        q_cmd = q_start + (q_target - q_start) * cosine_blend(ratio)
        reply = bus.transact(
            motor_id,
            q=q_cmd,
            dq=0.0,
            kp=args.kp,
            kd=args.kd,
            tau=0.0,
        )
        last_q = unwrap_near(reply.q, q_cmd)
        error_output_deg = math.degrees(q_cmd - last_q) / gear
        if abs(error_output_deg) > args.max_tracking_error_deg:
            excessive_error_count += 1
        else:
            excessive_error_count = 0
        if excessive_error_count >= 5:
            raise RuntimeError(
                f"tracking error stayed above limit: {error_output_deg:+.2f} deg"
            )
        if ratio >= 1.0:
            break
        time.sleep(args.dt)
    return last_q


def hold(bus, motor_id, q_target, gear, args):
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
        error_output_deg = math.degrees(q_target - last_q) / gear
        if abs(error_output_deg) > args.max_tracking_error_deg:
            raise RuntimeError(
                f"hold tracking error exceeds limit: {error_output_deg:+.2f} deg"
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
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--motor", choices=RL_MOTOR_ORDER, required=True)
    parser.add_argument(
        "--motor-id",
        type=int,
        help="Override provisional hip=0, thigh=1, knee=2 mapping.",
    )
    parser.add_argument(
        "--bus-ids",
        nargs="+",
        type=int,
        default=[0, 1, 2],
        help="All motor IDs on this bus; all receive STOP before and after test.",
    )
    parser.add_argument("--step-deg", type=float, default=2.0)
    parser.add_argument("--direction", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--kp", type=float, default=0.10)
    parser.add_argument("--kd", type=float, default=0.02)
    parser.add_argument("--ramp-sec", type=float, default=2.0)
    parser.add_argument("--hold-sec", type=float, default=0.5)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--max-tracking-error-deg", type=float, default=8.0)
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()

    numeric = (
        args.step_deg,
        args.kp,
        args.kd,
        args.ramp_sec,
        args.hold_sec,
        args.dt,
        args.max_tracking_error_deg,
    )
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if not 0.0 < args.step_deg <= 10.0:
        parser.error("step-deg must be in (0, 10]")
    if args.kp < 0.0 or args.kd < 0.0:
        parser.error("kp and kd must be non-negative")
    if args.ramp_sec <= 0.0 or args.dt <= 0.0 or args.hold_sec < 0.0:
        parser.error("ramp/dt must be positive and hold must be non-negative")

    motor_id = (
        DEFAULT_RL_MOTOR_IDS[args.motor]
        if args.motor_id is None
        else args.motor_id
    )
    if motor_id < 0 or motor_id > 14:
        parser.error("motor-id must be in 0..14")
    if any(value < 0 or value > 14 for value in args.bus_ids):
        parser.error("bus-ids must be in 0..14")
    if len(set(args.bus_ids)) != len(args.bus_ids):
        parser.error("bus-ids must not contain duplicates")
    bus_ids = sorted(set(args.bus_ids + [motor_id]))

    print("GO4 one bare-motor test")
    print(f"port={args.port} role={args.motor} id={motor_id}")
    print(
        f"sequence=current -> +{args.step_deg:.2f} deg -> current -> "
        f"-{args.step_deg:.2f} deg -> current"
    )
    print(
        f"kp={args.kp:.3f} kd={args.kd:.3f} "
        f"ramp={args.ramp_sec:.2f}s hold={args.hold_sec:.2f}s"
    )
    print("Angles are internal-reducer OUTPUT angles, not URDF joint angles.")

    if not args.enable_motion:
        print("DRY RUN only. Add --enable-motion after checking role and ID.")
        return 0

    sdk = import_unitree_sdk(args.sdk_path)
    bus = UnitreeDaisyChain(sdk, args.port)
    gear = bus.gear_ratio()
    print(f"SDK internal gear ratio={gear:.6f}")
    print(f"stopping all configured bus IDs before motion: {bus_ids}")
    bus.stop_many(bus_ids, repeats=3)
    q_origin = bus.read_mean_q(motor_id, samples=10)
    print(f"current rotor phase={q_origin:+.6f} rad")

    print("\nBefore continuing:")
    print("  1. The external 16:28 gear/parallelogram must be disconnected.")
    print("  2. The motor output must be free to rotate by at least +/-5 deg.")
    print("  3. No other process may use this RS485 port.")
    answer = input("Type YES to run this one motor: ").strip().upper()
    if answer != "YES":
        print("Cancelled; sending stop frames.")
        bus.stop_many(bus_ids, repeats=5)
        return 0

    q_last = q_origin
    result = 0
    try:
        for offset_deg in (
            args.step_deg,
            0.0,
            -args.step_deg,
            0.0,
        ):
            q_target = q_origin + math.radians(
                offset_deg * args.direction
            ) * gear
            current_output_deg = (
                math.degrees(q_last - q_origin) / gear * args.direction
            )
            print(
                f"move output {current_output_deg:+.2f} -> "
                f"{offset_deg:+.2f} deg"
            )
            q_last = move_segment(
                bus, motor_id, q_last, q_target, gear, args
            )
            q_last = hold(bus, motor_id, q_target, gear, args)
            reached_output_deg = (
                math.degrees(q_last - q_origin) / gear * args.direction
            )
            print(
                f"segment done: target={offset_deg:+.2f} deg, "
                f"actual={reached_output_deg:+.2f} deg, "
                f"error={offset_deg - reached_output_deg:+.2f} deg"
            )
    except KeyboardInterrupt:
        result = 130
        print("Interrupted: releasing motor.")
    except Exception as exc:
        result = 1
        print(f"FAULT: {exc}")
    finally:
        fade_error = None
        try:
            fade_stiffness(bus, motor_id, q_last, args)
        except Exception as exc:
            fade_error = exc
            print(f"stiffness fade failed: {exc}; still sending STOP frames")
        try:
            bus.stop_many(bus_ids, repeats=5)
            print("Stop replies confirmed for all configured bus IDs.")
            if fade_error is not None:
                result = 1
        except Exception as exc:
            result = 1
            print(f"STOP FAILED: {exc}; cut motor power immediately.")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
