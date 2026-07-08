#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from pathlib import Path


HOME_FILE = Path.home() / "single_calf_home.json"


def import_sdk(sdk_path):
    if sdk_path:
        sys.path.insert(0, str(Path(sdk_path).expanduser().resolve()))
    import unitree_actuator_sdk as sdk
    return sdk


def setup_cmd(sdk, cmd, motor_id):
    cmd.motorType = sdk.MotorType.GO_M8010_6
    cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
    cmd.id = motor_id


def send_cmd(sdk, serial, cmd, data, motor_id, q, dq, kp, kd, tau):
    setup_cmd(sdk, cmd, motor_id)
    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.q = q
    cmd.dq = dq
    cmd.kp = kp
    cmd.kd = kd
    cmd.tau = tau
    serial.sendRecv(cmd, data)
    return float(data.q), float(data.dq), int(data.merror)


def read_q(sdk, serial, cmd, data, motor_id, samples=20):
    vals = []
    for _ in range(samples):
        q, _, err = send_cmd(
            sdk, serial, cmd, data, motor_id,
            q=0.0, dq=0.0, kp=0.0, kd=0.01, tau=0.0
        )
        vals.append(q)
        time.sleep(0.01)
    return sum(vals) / len(vals), err


def save_home(path, port, motor_id, direction, gear, q_home):
    body = {
        "port": port,
        "id": motor_id,
        "direction": direction,
        "gear": gear,
        "q_home": q_home,
    }
    path.write_text(json.dumps(body, indent=2) + "\n", encoding="utf-8")


def load_home(path):
    return json.loads(path.read_text(encoding="utf-8"))


def move_to_angle(sdk, serial, cmd, data, motor_id, q_home, gear, direction, angle_deg, ramp_sec, kp, kd):
    q_now, _, _ = send_cmd(
        sdk, serial, cmd, data, motor_id,
        q=0.0, dq=0.0, kp=0.0, kd=0.01, tau=0.0
    )

    q_target = q_home + math.radians(angle_deg * direction) * gear
    t0 = time.time()

    while True:
        ratio = min((time.time() - t0) / ramp_sec, 1.0)
        ease = 0.5 - 0.5 * math.cos(math.pi * ratio)
        q_cmd = q_now + (q_target - q_now) * ease

        q_read, dq_read, err = send_cmd(
            sdk, serial, cmd, data, motor_id,
            q=q_cmd, dq=0.0, kp=kp, kd=kd, tau=0.0
        )

        joint_deg = math.degrees((q_read - q_home) / gear) * direction
        print(f"target={angle_deg:+.2f} deg, now={joint_deg:+.2f} deg, err={err}")

        if ratio >= 1.0:
            break
        time.sleep(0.01)


def check_angle_limit(angle_deg, min_deg, max_deg):
    if min_deg > max_deg:
        raise ValueError(f"invalid angle limit: min {min_deg} > max {max_deg}")
    if angle_deg < min_deg or angle_deg > max_deg:
        raise ValueError(
            f"angle {angle_deg:+.2f} deg is outside safe range "
            f"[{min_deg:+.2f}, {max_deg:+.2f}] deg"
        )


def soft_release(sdk, serial, cmd, data, motor_id):
    for i in range(80):
        fade = 1.0 - i / 80.0
        send_cmd(
            sdk, serial, cmd, data, motor_id,
            q=data.q, dq=0.0, kp=0.12 * fade, kd=0.02 * fade, tau=0.0
        )
        time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--direction", type=float, default=1.0)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--angle-deg", type=float)
    parser.add_argument("--min-deg", type=float, default=-140.0)
    parser.add_argument("--max-deg", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=0.15)
    parser.add_argument("--kd", type=float, default=0.025)
    parser.add_argument("--ramp-sec", type=float, default=1.0)
    args = parser.parse_args()

    if not args.calibrate and args.angle_deg is None:
        print("Use --calibrate first, or use --angle-deg ANGLE.")
        return

    if not args.calibrate:
        try:
            check_angle_limit(args.angle_deg, args.min_deg, args.max_deg)
        except ValueError as exc:
            print(f"Refusing to move: {exc}")
            print("For the current calf setup, use negative angles within [-140, 0] deg.")
            return

        if not HOME_FILE.exists():
            print(f"Missing {HOME_FILE}. Run --calibrate first.")
            return

    sdk = import_sdk(args.sdk_path)
    gear = float(sdk.queryGearRatio(sdk.MotorType.GO_M8010_6))

    serial = sdk.SerialPort(args.port)
    cmd = sdk.MotorCmd()
    data = sdk.MotorData()

    if args.calibrate:
        print("Put the calf motor at mechanical zero now.")
        reply = input("Type YES to save current position as calf zero: ").strip().upper()
        if reply != "YES":
            print("Cancelled.")
            return

        q_home, err = read_q(sdk, serial, cmd, data, args.id)
        save_home(HOME_FILE, args.port, args.id, args.direction, gear, q_home)
        print(f"saved {HOME_FILE}")
        print(f"q_home={q_home:.6f} rad, gear={gear:.6f}, err={err}")
        return

    home = load_home(HOME_FILE)
    print(f"home file: {HOME_FILE}")
    print(f"port={args.port}, id={args.id}, direction={args.direction:+.0f}")
    print(f"safe range=[{args.min_deg:+.2f}, {args.max_deg:+.2f}] deg")
    print(f"command angle={args.angle_deg:+.2f} deg")
    reply = input("Type YES to move calf motor: ").strip().upper()
    if reply != "YES":
        print("Cancelled.")
        return

    try:
        move_to_angle(
            sdk, serial, cmd, data, args.id,
            q_home=float(home["q_home"]),
            gear=float(home["gear"]),
            direction=args.direction,
            angle_deg=args.angle_deg,
            ramp_sec=args.ramp_sec,
            kp=args.kp,
            kd=args.kd,
        )
    finally:
        soft_release(sdk, serial, cmd, data, args.id)

    print("Done.")


if __name__ == "__main__":
    main()
