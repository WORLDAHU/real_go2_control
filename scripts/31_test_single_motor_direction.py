#!/usr/bin/env python3
import argparse
import json
import time
from urllib import request


MOTOR_NAMES = ("hip_motor", "thigh_motor", "calf_motor")


def post_json(url, body, timeout=2.0):
    data = json.dumps(body).encode("utf-8")
    req = request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with request.urlopen(req, timeout=timeout) as resp:
        return resp.read().decode("utf-8", errors="replace")


def make_command(args, motor_name, offset_deg):
    cmd = {
        "hip_motor": args.hip_zero_deg,
        "thigh_motor": args.thigh_zero_deg,
        "calf_motor": args.calf_zero_deg,
    }
    cmd[motor_name] += offset_deg
    return cmd


def main():
    parser = argparse.ArgumentParser(
        description="Conservative single-motor direction test: 0 -> +3 -> 0 -> -3 -> 0 deg."
    )
    parser.add_argument(
        "--motor",
        choices=MOTOR_NAMES,
        required=True,
        help="Only this motor will move.",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="Actually send commands to bridge. Without this, only print.",
    )
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:8765/set_motor_commands",
        help="Bridge endpoint URL.",
    )
    parser.add_argument("--amplitude-deg", type=float, default=3.0)
    parser.add_argument("--hold-sec", type=float, default=1.0)

    parser.add_argument("--hip-zero-deg", type=float, default=0.0)
    parser.add_argument("--thigh-zero-deg", type=float, default=0.0)
    parser.add_argument("--calf-zero-deg", type=float, default=0.0)

    args = parser.parse_args()

    sequence = [
        ("zero", 0.0),
        ("positive", +args.amplitude_deg),
        ("zero", 0.0),
        ("negative", -args.amplitude_deg),
        ("zero", 0.0),
    ]

    print("Single motor direction test")
    print(f"motor: {args.motor}")
    print(f"send: {args.send}")
    print(f"url: {args.url}")
    print("sequence: 0 -> +amp -> 0 -> -amp -> 0 deg")
    print()

    if not args.send:
        print("DRY RUN ONLY. Add --send when you are ready to send to bridge.")
        print()

    for label, offset_deg in sequence:
        cmd = make_command(args, args.motor, offset_deg)

        print(f"[{label}] offset={offset_deg:+.2f} deg")
        print(json.dumps(cmd, indent=2))

        if args.send:
            try:
                reply = post_json(args.url, cmd)
                if reply:
                    print(f"bridge reply: {reply}")
            except Exception as exc:
                print(f"ERROR sending command: {exc}")
                print("Stopping test.")
                return

        print()
        time.sleep(args.hold_sec)

    print("Test sequence finished.")


if __name__ == "__main__":
    main()