#!/usr/bin/env python3
import argparse
import math
import sys
import time
from pathlib import Path


def import_sdk(sdk_path):
    if sdk_path:
        sys.path.insert(0, str(Path(sdk_path).expanduser().resolve()))
    import unitree_actuator_sdk as sdk
    return sdk


def send_cmd(sdk, serial, cmd, data, motor_id, q, dq, kp, kd, tau):
    cmd.motorType = sdk.MotorType.GO_M8010_6
    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
    cmd.id = motor_id
    cmd.q = q
    cmd.dq = dq
    cmd.kp = kp
    cmd.kd = kd
    cmd.tau = tau
    serial.sendRecv(cmd, data)
    return data.q, data.dq, data.merror


def read_q(sdk, serial, cmd, data, motor_id, samples=20):
    vals = []
    for _ in range(samples):
        q, dq, err = send_cmd(
            sdk, serial, cmd, data, motor_id,
            q=0.0, dq=0.0, kp=0.0, kd=0.01, tau=0.0
        )
        vals.append(q)
        time.sleep(0.01)
    return sum(vals) / len(vals)


def move_to_offset(sdk, serial, cmd, data, motor_id, q_home, gear, direction, offset_deg, ramp_sec, kp, kd):
    q_target = q_home + math.radians(offset_deg * direction) * gear

    q_start, _, _ = send_cmd(
        sdk, serial, cmd, data, motor_id,
        q=0.0, dq=0.0, kp=0.0, kd=0.01, tau=0.0
    )

    t0 = time.time()
    while True:
        ratio = min((time.time() - t0) / ramp_sec, 1.0)
        ease = 0.5 - 0.5 * math.cos(math.pi * ratio)
        q_cmd = q_start + (q_target - q_start) * ease

        q_now, dq_now, err = send_cmd(
            sdk, serial, cmd, data, motor_id,
            q=q_cmd, dq=0.0, kp=kp, kd=kd, tau=0.0
        )

        joint_now_deg = math.degrees((q_now - q_home) / gear) * direction
        print(
            f"target={offset_deg:+.2f} deg, "
            f"now={joint_now_deg:+.2f} deg, "
            f"rotor_q={q_now:+.4f}, err={err}"
        )

        if err != 0:
            print("WARNING: motor reports non-zero error.")

        if ratio >= 1.0:
            break

        time.sleep(0.01)


def soft_release(sdk, serial, cmd, data, motor_id):
    print("soft release...")
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
    parser.add_argument("--port", default="/dev/ttyUSB3")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--direction", type=float, default=-1.0)
    parser.add_argument("--amplitude-deg", type=float, default=3.0)
    parser.add_argument("--hold-sec", type=float, default=1.0)
    parser.add_argument("--ramp-sec", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=0.12)
    parser.add_argument("--kd", type=float, default=0.02)
    parser.add_argument("--move", action="store_true")
    args = parser.parse_args()

    print("Single Unitree motor direct test")
    print(f"port={args.port}, id={args.id}, direction={args.direction:+.0f}")
    print("Default sequence: 0 -> +amp -> 0 -> -amp -> 0 deg")
    print()

    sdk = import_sdk(args.sdk_path)
    gear = float(sdk.queryGearRatio(sdk.MotorType.GO_M8010_6))
    serial = sdk.SerialPort(args.port)
    cmd = sdk.MotorCmd()
    data = sdk.MotorData()

    print(f"gear={gear:.6f}")
    print("Reading current rotor position as temporary zero...")
    q_home = read_q(sdk, serial, cmd, data, args.id)
    print(f"q_home={q_home:.6f} rad")
    print()

    if not args.move:
        print("READ ONLY. No motion commanded.")
        print("Add --move when you are ready.")
        return

    reply = input("Type YES to move this single motor by small angles: ").strip().upper()
    if reply != "YES":
        print("Cancelled. No motion commanded.")
        return

    sequence = [0.0, args.amplitude_deg, 0.0, -args.amplitude_deg, 0.0]

    try:
        for offset in sequence:
            print()
            print(f"moving to {offset:+.2f} deg")
            move_to_offset(
                sdk, serial, cmd, data, args.id,
                q_home=q_home,
                gear=gear,
                direction=args.direction,
                offset_deg=offset,
                ramp_sec=args.ramp_sec,
                kp=args.kp,
                kd=args.kd,
            )
            time.sleep(args.hold_sec)
    finally:
        soft_release(sdk, serial, cmd, data, args.id)

    print("Done.")


if __name__ == "__main__":
    main()
