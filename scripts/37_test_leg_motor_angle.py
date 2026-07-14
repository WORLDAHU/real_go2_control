#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from real_leg_adapter import RealLegCommandAdapter


HOME_FILE = os.path.expanduser("~/motor_home.json")
MAX_ABS_Q_HOME_RAD = 10000.0
TWO_PI = 2.0 * math.pi

# 与 33/32 一致：q_home 不是笼统的机械零位，而是在固定标定姿态记录的
# 编码器参考。单电机脚本也必须拒绝旧格式或错误姿态的 home 文件。
_CALIBRATION_MODEL = RealLegCommandAdapter()
EXPECTED_CALIBRATION_METADATA = {
    "hip_motor": {"common_deg": 0.0},
    "thigh_motor": {"common_deg": 90.0},
    "calf_motor": {
        "common_deg": _CALIBRATION_MODEL.fourbar.knee_pitch_home_deg,
        "crank_deg": _CALIBRATION_MODEL.fourbar.crank_home_deg,
    },
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


def validate_reply(ok, data, motor_id, allow_motor_fault=False):
    if not bool(ok):
        raise RuntimeError("sendRecv timeout/no reply")
    if not bool(data.correct):
        raise RuntimeError("invalid CRC/frame")
    if int(data.motor_id) != int(motor_id):
        raise RuntimeError(
            f"reply id={int(data.motor_id)}, expected {int(motor_id)}"
        )
    q = float(data.q)
    if not math.isfinite(q) or abs(q) > MAX_ABS_Q_HOME_RAD:
        raise RuntimeError(f"invalid rotor q={q!r}")
    if not allow_motor_fault and int(data.merror) != 0:
        raise RuntimeError(f"motor fault merror={int(data.merror)}")
    return q


def send_cmd(sdk, serial, cmd, data, motor_id, q, dq, kp, kd, tau):
    setup_cmd(sdk, cmd, motor_id)
    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.motorType = sdk.MotorType.GO_M8010_6
    cmd.q = q
    cmd.dq = dq
    cmd.kp = kp
    cmd.kd = kd
    cmd.tau = tau
    ok = serial.sendRecv(cmd, data)
    q_read = validate_reply(ok, data, motor_id)
    return q_read, float(data.dq), int(data.merror)


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
    ok = bool(serial.sendRecv(cmd, data))
    valid = (
        ok
        and bool(data.correct)
        and int(data.motor_id) == int(motor_id)
    )
    return {"reply_valid": valid, "merror": int(data.merror) if valid else None}


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


def validate_calibration_metadata(entry, motor_name):
    """校验文件声明；不能验证人工摆放的真实机械姿态。"""
    ref = entry.get("calibration_reference")
    if not isinstance(ref, dict):
        raise ValueError(
            f"{motor_name} home has no calibration_reference metadata. "
            "Re-run scripts/33_calibrate_motor_home.py after manually confirming the pose."
        )

    for key, expected in EXPECTED_CALIBRATION_METADATA[motor_name].items():
        actual = ref.get(key)
        if actual is None or abs(float(actual) - expected) > 1e-6:
            raise ValueError(
                f"{motor_name} calibration metadata[{key!r}]={actual!r}, "
                f"expected {expected}. The file metadata does not match current code."
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


def unwrap_near(angle_rad, reference_rad):
    return float(angle_rad) + round(
        (float(reference_rad) - float(angle_rad)) / TWO_PI
    ) * TWO_PI


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
        vals.append(q if not vals else unwrap_near(q, vals[-1]))
        time.sleep(0.01)
    q_mean = sum(vals) / len(vals)
    aligned_q_home = unwrap_near(q_home, q_mean)
    joint_deg = (
        rotor_to_joint(q_mean, aligned_q_home, gear) * cfg["direction"]
    )
    return joint_deg, err, aligned_q_home


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
    q_start = unwrap_near(q_start, q_home)
    last_q_read = q_start
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
        q_read = unwrap_near(q_read, q_cmd)
        last_q_read = q_read
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
        q_read = unwrap_near(q_read, q_target)
        last_q_read = q_read
        now_deg = rotor_to_joint(q_read, q_home, gear) * cfg["direction"]
        pos_err = angle_deg - now_deg
        print(
            f"hold target={angle_deg:+.2f} deg, now={now_deg:+.2f} deg, "
            f"pos_err={pos_err:+.2f} deg, err={err}"
        )
        time.sleep(0.05)
    return last_q_read


def soft_release(sdk, serial, cmd, data, motor_id, q_hold, kp, kd):
    print("soft release...")
    for i in range(80):
        fade = 1.0 - i / 80.0
        send_cmd(
            sdk,
            serial,
            cmd,
            data,
            motor_id,
            q=q_hold,
            dq=0.0,
            kp=kp * fade,
            kd=kd * fade,
            tau=0.0,
        )
        time.sleep(0.01)

    print("send motor stop mode...")
    return stop_only(sdk, serial, cmd, data, motor_id)


def stop_only(sdk, serial, cmd, data, motor_id, repeats=20):
    valid = 0
    zero_error = 0
    for _ in range(repeats):
        try:
            reply = send_stop(sdk, serial, cmd, data, motor_id)
            if reply["reply_valid"]:
                valid += 1
                zero_error += int(reply["merror"] == 0)
        except Exception as exc:
            print(f"stop send failed: {exc}")
        time.sleep(0.01)
    print(f"stop replies valid={valid}/{repeats}, zero_error={zero_error}/{repeats}")
    return valid == repeats and zero_error == repeats


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

    numeric = (args.angle_deg, args.kp, args.kd, args.ramp_sec, args.hold_sec)
    if not all(math.isfinite(value) for value in numeric):
        print("Refusing to run: all numeric arguments must be finite.")
        return 1
    if args.kp < 0.0 or args.kd < 0.0 or args.ramp_sec <= 0.0 or args.hold_sec < 0.0:
        print("Refusing to run: kp/kd/hold must be non-negative and ramp must be positive.")
        return 1

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
        validate_calibration_metadata(entry, args.motor)
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
    position_valid = False
    motion_confirmed = False
    had_fault = False
    result_code = 0
    release_q = None
    try:
        print(
            "注意：下面的当前位置读取会发送零刚度 FOC 查询帧；"
            "确认没有 bridge 或其他串口程序运行。"
        )
        current_deg, err, q_home = read_current_joint(
            sdk, serial, cmd, data, cfg, q_home, gear
        )
        position_valid = True
        print(
            f"current={current_deg:+.2f} deg, err={err}, "
            f"aligned_q_home={q_home:.6f} rad"
        )
        print()

        reply = input("Type YES to move this motor: ").strip().upper()
        if reply != "YES":
            print("Cancelled; sending mode=0 stop frames.")
        else:
            motion_confirmed = True
            release_q = move_to_angle(
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
    except Exception as exc:
        had_fault = True
        result_code = 1
        print(f"FAULT: {exc}")
    finally:
        cleanup_ok = False
        if position_valid and motion_confirmed and not had_fault and release_q is not None:
            try:
                cleanup_ok = soft_release(
                    sdk, serial, cmd, data, cfg["id"], release_q, args.kp, args.kd
                )
            except Exception as exc:
                print(f"soft release failed: {exc}; switching to mode=0 only")
                cleanup_ok = stop_only(sdk, serial, cmd, data, cfg["id"])
        else:
            cleanup_ok = stop_only(sdk, serial, cmd, data, cfg["id"])
        if not cleanup_ok:
            result_code = 1
            print("WARNING: stop replies were not fully confirmed; cut motor power.")

    if motion_confirmed and result_code == 0:
        print("Done; stop replies confirmed. Physical power state is not verified.")
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
