#!/usr/bin/env python3
"""Capture a temporary assembled-leg reference pose without commanding motion."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_leg_adapter import DEFAULT_RL_MOTOR_IDS, RL_MOTOR_ORDER
from go4_rl_kinematics import Go4RLKinematics, RL_JOINT_NAMES
from unitree_daisy_chain import UnitreeDaisyChain, import_unitree_sdk


SCHEMA = "go4_rl_relative_reference_v1"
DEFAULT_REFERENCE_FILE = os.path.expanduser("~/go4_rl_reference.json")
DEFAULT_URDF = Path(__file__).resolve().parents[1] / "models" / "go4" / "GO4.urdf"


def read_stable(bus, motor_id, settle_timeout_sec):
    deadline = time.monotonic() + settle_timeout_sec
    while True:
        try:
            return bus.read_mean_q(motor_id, samples=20)
        except RuntimeError as exc:
            if "unstable rotor readings" not in str(exc) or time.monotonic() >= deadline:
                raise
            print(f"  waiting for id={motor_id} to settle: {exc}")
            time.sleep(0.25)


def atomic_write_json(path, payload):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--hip-id", type=int, default=DEFAULT_RL_MOTOR_IDS["hip"])
    parser.add_argument("--thigh-id", type=int, default=DEFAULT_RL_MOTOR_IDS["thigh"])
    parser.add_argument("--knee-id", type=int, default=DEFAULT_RL_MOTOR_IDS["knee"])
    parser.add_argument("--settle-timeout-sec", type=float, default=5.0)
    parser.add_argument("--output", default=DEFAULT_REFERENCE_FILE)
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    args = parser.parse_args()

    urdf_path = Path(args.urdf).expanduser().resolve()
    kinematics = Go4RLKinematics.from_urdf(urdf_path)
    urdf_reference_deg = {
        name: math.degrees(value)
        for name, value in zip(RL_JOINT_NAMES, kinematics.retracted_q())
    }

    motor_ids = {
        "hip": args.hip_id,
        "thigh": args.thigh_id,
        "knee": args.knee_id,
    }
    if len(set(motor_ids.values())) != 3:
        parser.error("hip/thigh/knee IDs must be unique")
    if any(value < 0 or value > 14 for value in motor_ids.values()):
        parser.error("motor IDs must be in 0..14")
    if not math.isfinite(args.settle_timeout_sec) or args.settle_timeout_sec <= 0:
        parser.error("settle-timeout-sec must be positive and finite")

    print("GO4 RL assembled-leg URDF reference capture")
    print("This script reads encoders only; it never commands position motion.")
    print("The encoder phase will be tied to this declared URDF pose:")
    for name in RL_JOINT_NAMES:
        print(f"  {name}={urdf_reference_deg[name]:+.6f} deg")
    print()
    print("Before continuing:")
    print("  1. Install the transmission and support the leg against gravity.")
    print("  2. Set hip neutral, thigh to its URDF upper reference, and")
    print("     fully retract the calf to its URDF lower reference.")
    print("  3. Keep hands clear and do not move the joints during capture.")
    print("  4. Make sure no other process owns this RS485 port.")
    answer = input("Type CAPTURE to record this relative reference: ").strip().upper()
    if answer != "CAPTURE":
        print("Cancelled; no file was written.")
        return 0

    sdk = import_unitree_sdk(args.sdk_path)
    bus = UnitreeDaisyChain(sdk, args.port)
    ids_in_order = [motor_ids[role] for role in RL_MOTOR_ORDER]
    readings = {}
    result = 0
    try:
        bus.stop_many(ids_in_order, repeats=3)
        for role in RL_MOTOR_ORDER:
            motor_id = motor_ids[role]
            print(f"reading {role}: id={motor_id}")
            readings[role] = read_stable(
                bus, motor_id, args.settle_timeout_sec
            )
            bus.stop(motor_id)
            print(f"  q_reference_phase={readings[role]:+.6f} rad")
    except Exception as exc:
        result = 1
        print(f"CAPTURE FAILED: {exc}")
    finally:
        try:
            bus.stop_many(ids_in_order, repeats=5)
            print("Stop replies confirmed for all configured bus IDs.")
        except Exception as exc:
            result = 1
            print(f"STOP FAILED: {exc}; cut motor power immediately.")

    if result != 0:
        print("Reference file was not changed.")
        return result

    payload = {
        "schema": SCHEMA,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "purpose": "GO4 RL fully-retracted URDF pose encoder-phase reference",
        "urdf": {
            "path_at_capture": str(urdf_path),
            "sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
            "joint_reference_deg": urdf_reference_deg,
        },
        "port": args.port,
        "motor_type": "GO_M8010_6",
        "internal_gear_ratio": bus.gear_ratio(),
        "knee_external_transmission": {
            "motor_teeth": 16,
            "knee_teeth": 28,
            "parallelogram_ratio": 1.0,
        },
        "motors": {
            role: {
                "id": motor_ids[role],
                "q_reference_phase_rad": readings[role],
            }
            for role in RL_MOTOR_ORDER
        },
    }
    output_path = atomic_write_json(args.output, payload)
    print(f"saved: {output_path}")
    print("This reuses the GO-M8010-6 rotor-phase/gear-ratio calibration method")
    print("from script 33, while the declared joint angles come from GO4.urdf.")
    print("Keep the leg near this exact pose before running script 44.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
