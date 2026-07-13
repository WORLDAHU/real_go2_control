#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


HOME_FILE = os.path.expanduser("~/motor_home.json")
MAX_ABS_Q_HOME_RAD = 10000.0

# 与 33/32 一致：q_home 不是笼统的机械零位，而是在固定标定姿态记录的
# 编码器参考。单电机脚本也必须拒绝旧格式或错误姿态的 home 文件。
EXPECTED_CALIBRATION_REFERENCE = {
    "hip_motor": {"common_deg": 0.0},
    "thigh_motor": {"common_deg": 90.0},
    "calf_motor": {"common_deg": -160.59, "crank_deg": 10.0},
}

DEFAULT_MOTORS = {
    "hip_motor": {
        "label": "hip",
        "port": "/dev/ttyUSB2",
        "id": 2,
        "direction": 1.0,
        "min_deg": -30.0,
        "max_deg": 30.0,
    },
    "thigh_motor": {
        "label": "thigh",
        "port": "/dev/ttyUSB1",
        "id": 0,
        "direction": 1.0,
        # 这里的 angle-deg 是 bridge 坐标，不是 common thigh_pitch。
        # 上电标定时大腿水平（common=+90）对应 bridge=0；若 common 安全
        # 范围为 [-30, +90]，则 bridge 安全范围必须是 [-120, 0]。
        "min_deg": -120.0,
        "max_deg": 0.0,
    },
    "calf_motor": {
        "label": "calf",
        "port": "/dev/ttyUSB0",
        "id": 1,
        "direction": 1.0,
        "min_deg": -180.0,
        "max_deg": 0.0,
    },
}


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
    cmd.motorType = sdk.MotorType.GO_M8010_6
    cmd.q = q
    cmd.dq = dq
    cmd.kp = kp
    cmd.kd = kd
    cmd.tau = tau
    serial.sendRecv(cmd, data)
    return float(data.q), float(data.dq), int(data.merror)


def send_stop(sdk, serial, cmd, data, motor_id):
    setup_cmd(sdk, cmd, motor_id)
    stop_mode = getattr(sdk.MotorMode, "STOP", None)
    if stop_mode is not None:
        try:
            cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, stop_mode)
        except Exception:
            cmd.mode = 0
    else:
        cmd.mode = 0

    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.motorType = sdk.MotorType.GO_M8010_6
    cmd.q = 0.0
    cmd.dq = 0.0
    cmd.kp = 0.0
    cmd.kd = 0.0
    cmd.tau = 0.0
    serial.sendRecv(cmd, data)
    return int(data.merror)


def load_home(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def find_home_entry(home, motor_name, cfg):
    if motor_name in home:
        return home[motor_name]

    for value in home.values():
        if value.get("port") == cfg["port"] and int(value.get("id")) == int(cfg["id"]):
            return value

    raise KeyError(
        f"cannot find home for {motor_name}: port={cfg['port']} id={cfg['id']}"
    )


def validate_calibration_reference(entry, motor_name):
    """拒绝不带固定标定姿态元数据的旧 q_home 文件。"""
    ref = entry.get("calibration_reference")
    if not isinstance(ref, dict):
        raise ValueError(
            f"{motor_name} home has no calibration_reference. "
            "Re-run scripts/33_calibrate_motor_home.py at the fixed calibration pose."
        )

    for key, expected in EXPECTED_CALIBRATION_REFERENCE[motor_name].items():
        actual = ref.get(key)
        if actual is None or abs(float(actual) - expected) > 1e-6:
            raise ValueError(
                f"{motor_name} home calibration_reference[{key!r}]={actual!r}, "
                f"expected {expected}. Re-run scripts/33_calibrate_motor_home.py "
                "at the fixed calibration pose."
            )


def validate_home_numeric(entry, motor_name):
    """拒绝超时或坏帧留下的垃圾 q_home/gear。"""
    q_home = float(entry.get("q_home", float("nan")))
    if not math.isfinite(q_home) or abs(q_home) > MAX_ABS_Q_HOME_RAD:
        raise ValueError(f"{motor_name} invalid q_home={q_home!r}")

    gear = float(entry.get("gear", float("nan")))
    if not math.isfinite(gear) or gear <= 0.0:
        raise ValueError(f"{motor_name} invalid gear={gear!r}")


def check_angle_limit(angle_deg, min_deg, max_deg):
    if min_deg > max_deg:
        raise ValueError(f"invalid angle limit: min {min_deg} > max {max_deg}")
    if angle_deg < min_deg or angle_deg > max_deg:
        raise ValueError(
            f"angle {angle_deg:+.2f} deg is outside safe range "
            f"[{min_deg:+.2f}, {max_deg:+.2f}] deg"
        )


def joint_to_rotor(joint_deg, q_home, gear):
    return q_home + math.radians(joint_deg) * gear


def rotor_to_joint(rotor_rad, q_home, gear):
    return math.degrees((rotor_rad - q_home) / gear)


def read_current_joint(sdk, serial, cmd, data, cfg, q_home, gear, samples=10):
    vals = []
    for _ in range(samples):
        q, _, err = send_cmd(
            sdk,
            serial,
            cmd,
            data,
            cfg["id"],
            q=0.0,
            dq=0.0,
            kp=0.0,
            kd=0.01,
            tau=0.0,
        )
        vals.append(q)
        time.sleep(0.01)
    joint_deg = rotor_to_joint(sum(vals) / len(vals), q_home, gear) * cfg["direction"]
    return joint_deg, err


def move_to_angle(
    sdk,
    serial,
    cmd,
    data,
    cfg,
    q_home,
    gear,
    angle_deg,
    ramp_sec,
    hold_sec,
    kp,
    kd,
):
    q_start, _, _ = send_cmd(
        sdk,
        serial,
        cmd,
        data,
        cfg["id"],
        q=0.0,
        dq=0.0,
        kp=0.0,
        kd=0.01,
        tau=0.0,
    )
    target_joint_for_rotor = angle_deg * cfg["direction"]
    q_target = joint_to_rotor(target_joint_for_rotor, q_home, gear)

    t0 = time.time()
    while True:
        ratio = min((time.time() - t0) / ramp_sec, 1.0)
        ease = 0.5 - 0.5 * math.cos(math.pi * ratio)
        q_cmd = q_start + (q_target - q_start) * ease

        q_read, _, err = send_cmd(
            sdk,
            serial,
            cmd,
            data,
            cfg["id"],
            q=q_cmd,
            dq=0.0,
            kp=kp,
            kd=kd,
            tau=0.0,
        )
        now_deg = rotor_to_joint(q_read, q_home, gear) * cfg["direction"]
        pos_err = angle_deg - now_deg
        print(
            f"target={angle_deg:+.2f} deg, now={now_deg:+.2f} deg, "
            f"pos_err={pos_err:+.2f} deg, err={err}"
        )

        if ratio >= 1.0:
            break
        time.sleep(0.01)

    hold_until = time.time() + hold_sec
    while time.time() < hold_until:
        q_read, _, err = send_cmd(
            sdk,
            serial,
            cmd,
            data,
            cfg["id"],
            q=q_target,
            dq=0.0,
            kp=kp,
            kd=kd,
            tau=0.0,
        )
        now_deg = rotor_to_joint(q_read, q_home, gear) * cfg["direction"]
        pos_err = angle_deg - now_deg
        print(
            f"hold target={angle_deg:+.2f} deg, now={now_deg:+.2f} deg, "
            f"pos_err={pos_err:+.2f} deg, err={err}"
        )
        time.sleep(0.05)


def soft_release(sdk, serial, cmd, data, motor_id, kp, kd):
    print("soft release...")
    for i in range(80):
        fade = 1.0 - i / 80.0
        send_cmd(
            sdk,
            serial,
            cmd,
            data,
            motor_id,
            q=data.q,
            dq=0.0,
            kp=kp * fade,
            kd=kd * fade,
            tau=0.0,
        )
        time.sleep(0.01)

    print("send motor stop mode...")
    for _ in range(20):
        send_stop(sdk, serial, cmd, data, motor_id)
        time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument("--home-file", default=HOME_FILE)
    parser.add_argument(
        "--motor",
        choices=sorted(DEFAULT_MOTORS.keys()),
        required=True,
        help="Which real motor to test.",
    )
    parser.add_argument("--angle-deg", type=float, required=True)
    parser.add_argument("--min-deg", type=float)
    parser.add_argument("--max-deg", type=float)
    parser.add_argument("--kp", type=float, default=0.15)
    parser.add_argument("--kd", type=float, default=0.025)
    parser.add_argument("--ramp-sec", type=float, default=1.0)
    parser.add_argument("--hold-sec", type=float, default=1.0)
    args = parser.parse_args()

    cfg = dict(DEFAULT_MOTORS[args.motor])
    if args.min_deg is not None:
        cfg["min_deg"] = args.min_deg
    if args.max_deg is not None:
        cfg["max_deg"] = args.max_deg

    try:
        check_angle_limit(args.angle_deg, cfg["min_deg"], cfg["max_deg"])
    except ValueError as exc:
        print(f"Refusing to move {args.motor}: {exc}")
        return 1

    if not os.path.exists(args.home_file):
        print(f"Missing {args.home_file}. Run scripts/33_calibrate_motor_home.py first.")
        return 1

    home = load_home(args.home_file)
    entry = find_home_entry(home, args.motor, cfg)
    try:
        validate_calibration_reference(entry, args.motor)
        validate_home_numeric(entry, args.motor)
    except ValueError as exc:
        print(f"Refusing to move {args.motor}: {exc}")
        return 1
    q_home = float(entry["q_home"])
    gear = float(entry.get("gear", 0.0))

    sdk = import_sdk(args.sdk_path)
    if gear == 0.0:
        gear = float(sdk.queryGearRatio(sdk.MotorType.GO_M8010_6))

    print("Real leg single motor angle test")
    print(f"motor={args.motor} label={cfg['label']}")
    print(f"port={cfg['port']} id={cfg['id']} direction={cfg['direction']:+.0f}")
    print(f"safe range=[{cfg['min_deg']:+.2f}, {cfg['max_deg']:+.2f}] deg")
    print(f"command angle={args.angle_deg:+.2f} deg")
    print(f"home file={args.home_file}")
    print(f"q_home={q_home:.6f} rad, gear={gear:.6f}")

    serial = sdk.SerialPort(cfg["port"])
    cmd = sdk.MotorCmd()
    data = sdk.MotorData()

    current_deg, err = read_current_joint(
        sdk, serial, cmd, data, cfg, q_home, gear
    )
    print(f"current={current_deg:+.2f} deg, err={err}")
    print()

    reply = input("Type YES to move this motor: ").strip().upper()
    if reply != "YES":
        print("Cancelled.")
        return 0

    try:
        move_to_angle(
            sdk=sdk,
            serial=serial,
            cmd=cmd,
            data=data,
            cfg=cfg,
            q_home=q_home,
            gear=gear,
            angle_deg=args.angle_deg,
            ramp_sec=args.ramp_sec,
            hold_sec=args.hold_sec,
            kp=args.kp,
            kd=args.kd,
        )
    finally:
        soft_release(sdk, serial, cmd, data, cfg["id"], args.kp, args.kd)

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
