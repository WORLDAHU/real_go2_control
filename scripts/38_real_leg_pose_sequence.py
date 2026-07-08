#!/usr/bin/env python3
import argparse
import json
import time
import urllib.error
import urllib.request


MOTOR_LIMITS = {
    "hip_motor": (-30.0, 30.0),
    "thigh_motor": (-30.0, 90.0),
    "calf_motor": (-180.0, 0.0),
}


PRESETS = {
    # First real-leg coordination test. This is intentionally not a jump.
    "micro": [
        {"name": "neutral", "hip": 0.0, "thigh": 0.0, "calf": -20.0, "move": 1.0, "hold": 1.0},
        {"name": "load", "hip": 0.0, "thigh": 8.0, "calf": -45.0, "move": 1.2, "hold": 1.0},
        {"name": "extend", "hip": 0.0, "thigh": -5.0, "calf": -80.0, "move": 1.2, "hold": 1.0},
        {"name": "load", "hip": 0.0, "thigh": 8.0, "calf": -45.0, "move": 1.2, "hold": 0.6},
        {"name": "neutral", "hip": 0.0, "thigh": 0.0, "calf": -20.0, "move": 1.0, "hold": 1.0},
    ],
    # Still slow, but closer to a push preview. Use only after micro is clean.
    "push_preview": [
        {"name": "neutral", "hip": 0.0, "thigh": 0.0, "calf": -30.0, "move": 1.0, "hold": 0.8},
        {"name": "deeper_load", "hip": 0.0, "thigh": 20.0, "calf": -80.0, "move": 1.5, "hold": 0.8},
        {"name": "slow_push", "hip": 0.0, "thigh": -15.0, "calf": -150.0, "move": 1.0, "hold": 0.8},
        {"name": "recover", "hip": 0.0, "thigh": 5.0, "calf": -70.0, "move": 1.2, "hold": 0.8},
        {"name": "neutral", "hip": 0.0, "thigh": 0.0, "calf": -30.0, "move": 1.0, "hold": 1.0},
    ],
}


def request_json(method, url, body=None, timeout=2.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def command_from_pose(pose):
    return {
        "hip_motor": float(pose["hip"]),
        "thigh_motor": float(pose["thigh"]),
        "calf_motor": float(pose["calf"]),
    }


def validate_command(cmd):
    for name, value in cmd.items():
        lower, upper = MOTOR_LIMITS[name]
        if value < lower or value > upper:
            raise ValueError(
                f"{name}={value:+.2f} deg outside [{lower:+.2f}, {upper:+.2f}] deg"
            )


def lerp(a, b, ratio):
    return a + (b - a) * ratio


def ease_in_out(ratio):
    # Smoothstep: zero velocity at both ends.
    ratio = max(0.0, min(1.0, ratio))
    return ratio * ratio * (3.0 - 2.0 * ratio)


def post_command(base_url, cmd):
    validate_command(cmd)
    return request_json("POST", f"{base_url}/set_motor_commands", cmd)


def get_status(base_url):
    return request_json("GET", f"{base_url}/status")


def run_pose_sequence(base_url, poses, dt, repeat):
    status = get_status(base_url)
    if not status.get("ok"):
        raise RuntimeError(f"bridge status is not ok: {status}")
    if not status.get("motors_ready"):
        raise RuntimeError(f"bridge motors are not ready: {status}")

    current = status.get("target_deg") or status.get("current_deg")
    if not current:
        current = {"hip_motor": 0.0, "thigh_motor": 0.0, "calf_motor": 0.0}

    current = {name: float(current[name]) for name in MOTOR_LIMITS}
    print("bridge status:")
    print(json.dumps(status, indent=2, ensure_ascii=False))
    print()

    for cycle in range(repeat):
        print(f"sequence cycle {cycle + 1}/{repeat}")
        for pose in poses:
            target = command_from_pose(pose)
            validate_command(target)
            move_time = max(float(pose.get("move", 1.0)), dt)
            hold_time = max(float(pose.get("hold", 0.0)), 0.0)
            steps = max(1, int(move_time / dt))

            print(
                f"  -> {pose.get('name', 'pose')}: "
                f"hip={target['hip_motor']:+.1f}, "
                f"thigh={target['thigh_motor']:+.1f}, "
                f"calf={target['calf_motor']:+.1f}"
            )

            for step in range(1, steps + 1):
                r = ease_in_out(step / steps)
                cmd = {
                    name: lerp(current[name], target[name], r)
                    for name in MOTOR_LIMITS
                }
                post_command(base_url, cmd)
                time.sleep(dt)

            current = target

            t_end = time.time() + hold_time
            while time.time() < t_end:
                post_command(base_url, current)
                time.sleep(dt)


def load_custom_poses(path):
    with open(path, "r", encoding="utf-8") as f:
        poses = json.load(f)
    if not isinstance(poses, list):
        raise ValueError("custom pose file must contain a JSON list")
    for pose in poses:
        for key in ("hip", "thigh", "calf"):
            if key not in pose:
                raise ValueError(f"custom pose missing key: {key}")
    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge-url", default="http://127.0.0.1:8765")
    parser.add_argument("--preset", choices=sorted(PRESETS), default="micro")
    parser.add_argument("--custom-json", help="JSON list of poses with hip/thigh/calf/move/hold fields.")
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.repeat <= 0:
        raise ValueError("--repeat must be positive")

    poses = load_custom_poses(args.custom_json) if args.custom_json else PRESETS[args.preset]

    print("Real leg pose sequence")
    print(f"bridge: {args.bridge_url}")
    print(f"preset: {args.preset if not args.custom_json else args.custom_json}")
    print("This script sends slow bridge commands; it does not talk to motors directly.")
    print("Keep the leg supported and be ready to stop the bridge.")
    print()
    for pose in poses:
        cmd = command_from_pose(pose)
        validate_command(cmd)
        print(
            f"  {pose.get('name', 'pose'):>12s}: "
            f"hip={cmd['hip_motor']:+.1f}, "
            f"thigh={cmd['thigh_motor']:+.1f}, "
            f"calf={cmd['calf_motor']:+.1f}, "
            f"move={float(pose.get('move', 1.0)):.2f}s, "
            f"hold={float(pose.get('hold', 0.0)):.2f}s"
        )
    print()

    if not args.yes:
        reply = input("Type YES to run this slow sequence: ").strip().upper()
        if reply != "YES":
            print("Cancelled.")
            return 0

    try:
        run_pose_sequence(args.bridge_url, poses, args.dt, args.repeat)
    except urllib.error.URLError as exc:
        print(f"Cannot reach bridge: {exc}")
        return 1

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
