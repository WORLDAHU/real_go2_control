#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_MOTORS = {
    "hip_motor": {
        "label": "hip",
        "port": "/dev/ttyUSB0",
        "id": 2,
        "direction": 1.0,
    },
    "thigh_motor": {
        "label": "thigh",
        "port": "/dev/ttyUSB2",
        "id": 0,
        "direction": 1.0,
    },
    "calf_motor": {
        "label": "calf",
        "port": "/dev/ttyUSB3",
        "id": 1,
        "direction": -1.0,
    },
}

HOME_FILE = os.path.expanduser("~/motor_home.json")


def import_sdk(sdk_path):
    if sdk_path:
        sys.path.insert(0, str(Path(sdk_path).expanduser().resolve()))

    import unitree_actuator_sdk as sdk

    return sdk


def read_current_rotor(sdk, port, motor_id, samples, dt):
    serial = sdk.SerialPort(port)
    cmd = sdk.MotorCmd()
    data = sdk.MotorData()

    cmd.motorType = sdk.MotorType.GO_M8010_6
    cmd.mode = sdk.queryMotorMode(
        sdk.MotorType.GO_M8010_6,
        sdk.MotorMode.FOC,
    )
    cmd.id = int(motor_id)
    cmd.q = 0.0
    cmd.dq = 0.0
    cmd.kp = 0.0
    cmd.kd = 0.01
    cmd.tau = 0.0

    vals = []
    for _ in range(samples):
        data.motorType = sdk.MotorType.GO_M8010_6
        cmd.motorType = sdk.MotorType.GO_M8010_6
        serial.sendRecv(cmd, data)
        vals.append(float(data.q))
        time.sleep(dt)

    return sum(vals) / len(vals), data


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Read current Unitree motor rotor positions and save them as this power-on home."
    )
    parser.add_argument(
        "--sdk-path",
        default=None,
        help="Folder containing unitree_actuator_sdk.py or compiled SDK module.",
    )
    parser.add_argument(
        "--home-file",
        default=HOME_FILE,
        help="Where to save the home JSON. Default: ~/motor_home.json",
    )
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the confirmation prompt after manually placing the leg at zero.",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()

    if args.samples <= 0:
        raise ValueError("--samples must be positive")
    if args.dt < 0.0:
        raise ValueError("--dt must be non-negative")

    print("Unitree motor home calibration")
    print("This only reads rotor position; it does not command motion.")
    print()
    print("Before continuing:")
    print("  1. Power the motors and connect USB adapters.")
    print("  2. Put hip/thigh/calf at your mechanical zero by hand.")
    print("  3. Make sure no bridge with --enable-motors is running.")
    print()

    if not args.yes:
        reply = input("Type YES after the leg is at mechanical zero: ").strip()
        if reply != "YES":
            print("Cancelled. No file written.")
            return 1

    sdk = import_sdk(args.sdk_path)
    gear = float(sdk.queryGearRatio(sdk.MotorType.GO_M8010_6))
    print(f"gear ratio: {gear:.6f}")
    print()

    home = {}
    for name, cfg in DEFAULT_MOTORS.items():
        print(
            f"reading {name}: port={cfg['port']} "
            f"id={cfg['id']} dir={cfg['direction']:+.0f}"
        )
        q_home, data = read_current_rotor(
            sdk=sdk,
            port=cfg["port"],
            motor_id=cfg["id"],
            samples=args.samples,
            dt=args.dt,
        )
        home[name] = {
            "port": cfg["port"],
            "id": int(cfg["id"]),
            "direction": float(cfg["direction"]),
            "q_home": q_home,
            "gear": gear,
            "motor_type": "GO_M8010_6",
            "calibrated_at": datetime.now().isoformat(timespec="seconds"),
        }
        print(f"  q_home={q_home:.6f} rad, merror={getattr(data, 'merror', 'unknown')}")

    home_path = Path(args.home_file).expanduser()
    home_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = home_path.with_suffix(home_path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(home, indent=2) + "\n", encoding="utf-8")
    tmp_path.replace(home_path)

    print()
    print(f"saved: {home_path}")
    print("Next: start the bridge with --enable-motors, then test one motor with +/-3 deg.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
