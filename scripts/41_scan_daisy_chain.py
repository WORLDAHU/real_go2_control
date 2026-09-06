#!/usr/bin/env python3
"""Discover responding Unitree motor IDs without commanding motion."""

import argparse
import sys
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_leg_adapter import DEFAULT_RL_MOTOR_IDS, RL_MOTOR_ORDER
from unitree_daisy_chain import UnitreeDaisyChain, import_unitree_sdk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--ids", nargs="+", type=int, default=[0, 1, 2])
    args = parser.parse_args()

    if any(motor_id < 0 or motor_id > 14 for motor_id in args.ids):
        parser.error("motor IDs must be in the SDK range 0..14")
    if len(set(args.ids)) != len(args.ids):
        parser.error("motor IDs must not contain duplicates")

    print("GO4 Unitree daisy-chain read-only scan")
    print(f"port={args.port} ids={args.ids}")
    print("This sends zero-stiffness, zero-torque FOC query frames only.")
    print("Make sure no other bridge or SDK process owns this serial port.\n")

    sdk = import_unitree_sdk(args.sdk_path)
    bus = UnitreeDaisyChain(sdk, args.port)
    responders = []
    stop_failures = []
    for motor_id in args.ids:
        try:
            reply = bus.query(motor_id, allow_fault=True)
        except Exception as exc:
            print(f"id={motor_id:2d}: NO REPLY ({exc})")
            continue
        responders.append(motor_id)
        print(
            f"id={motor_id:2d}: OK q_rotor={reply.q:+.6f} rad "
            f"dq={reply.dq:+.6f} rad/s merror={reply.merror}"
        )
        try:
            bus.stop(motor_id)
        except Exception as exc:
            stop_failures.append(motor_id)
            print(f"id={motor_id:2d}: WARNING stop reply failed ({exc})")

    expected = [DEFAULT_RL_MOTOR_IDS[name] for name in RL_MOTOR_ORDER]
    print(f"\nresponders={responders}")
    print(
        "provisional role mapping: "
        + ", ".join(
            f"{name}=id{DEFAULT_RL_MOTOR_IDS[name]}" for name in RL_MOTOR_ORDER
        )
    )
    if sorted(responders) != sorted(expected):
        print("WARNING: replies do not match provisional IDs 0/1/2.")
        print("Use the responding IDs explicitly during the one-motor test.")
        return 1
    if stop_failures:
        print(f"WARNING: stop was not confirmed for IDs {stop_failures}.")
        return 1
    print("Scan passed. Physical daisy-chain order still does not prove role-to-ID mapping.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
