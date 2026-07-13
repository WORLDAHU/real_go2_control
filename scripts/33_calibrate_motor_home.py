#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_MOTORS = {
    "hip_motor": {
        "label": "hip",
        "port": "/dev/ttyUSB2",
        "id": 2,
        "direction": 1.0,
    },
    "thigh_motor": {
        "label": "thigh",
        "port": "/dev/ttyUSB1",
        "id": 0,
        "direction": 1.0,
    },
    "calf_motor": {
        "label": "calf",
        "port": "/dev/ttyUSB0",
        "id": 1,
        "direction": 1.0,
    },
}

HOME_FILE = os.path.expanduser("~/motor_home.json")
MAX_ABS_ROTOR_RAD = 10000.0
MAX_SAMPLE_SPREAD_RAD = 0.10
MIN_VALID_REPLY_RATIO = 0.80


# ============================================================================
# GO2 左后腿实机：上电标定姿态与“零点”约定
# ============================================================================
#
# 本项目同时存在仿真/RL、机械关节、真实电机编码器和四连杆小腿四个层级。
# 因此必须区分下列概念，不能把它们都称为“零点”。
#
# 1. 电机编码器角 q（rad）
#    GO-M8010-6 返回的原始转子位置 data.q。它不是直接可用的机械关节角。
#
# 2. 上电标定参考 q_home（rad）
#    本脚本在“固定标定姿态”读到的 q。它只是本次上电的软件参考原点，
#    不是电机出厂零点，也不等于 common 机械关节零位。
#
#    bridge 对 bridge_cmd_deg 的底层换算为：
#      q_target = q_home + radians(bridge_cmd_deg * direction) * gear_ratio
#
#    所以 bridge_cmd_deg=0 的含义是“回到本次标定姿态”。
#
# 3. common_joint_deg（deg）
#    固定的机械关节坐标，用于仿真、RL、运动学和机械限位讨论；它不随
#    每次上电或 q_home 改变。
#
# 4. bridge_cmd_deg（deg）
#    相对“本次上电标定姿态”的实机命令坐标；不是绝对机械关节角。
#
# 每次运行本脚本前，必须先释放电机，并手动把腿摆到同一套固定姿态：
#
#   hip_motor：无内外摆动
#       common hip_abduction =   0.00 deg
#       bridge hip_motor    =   0.00 deg
#       髋的机械零位与标定姿态恰好重合。
#
#   thigh_motor：大腿水平
#       common thigh_pitch  = +90.00 deg
#       bridge thigh_motor  =   0.00 deg
#       大腿水平是“上电标定姿态”，不是 common thigh_pitch 的机械零位。
#       例如 common +60 deg 对应 bridge -30 deg；common 0 deg 对应
#       bridge -90 deg。
#
#   calf_motor：小腿完全收缩的四连杆限位
#       crank_angle         = +10.00 deg
#       common knee_pitch   = -160.59 deg
#       bridge calf_motor   =   0.00 deg
#       knee_pitch=0 是理论机械参考，当前机构/安全行程无法达到；这不影响
#       标定。小腿仍须通过四连杆映射，不能直接做线性角度相减。
#
# 本脚本只记录上述固定姿态下的 q_home；它不改变 common_joint_deg 的定义，
# 也不重新拟合四连杆参数。
# ============================================================================
CALIBRATION_COMMON_REFERENCE = {
    "hip_motor": {
        "joint_name": "hip_abduction",
        "common_deg": 0.0,
        "description": "髋无内外摆动；common 机械零位。",
    },
    "thigh_motor": {
        "joint_name": "thigh_pitch",
        "common_deg": 90.0,
        "description": "大腿水平；上电标定参考，不是 common 机械零位。",
    },
    "calf_motor": {
        "joint_name": "knee_pitch",
        "common_deg": -160.59,
        "crank_deg": 10.0,
        "description": "小腿完全收缩限位；对应四连杆曲柄 10 度。",
    },
}


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
    reject_reasons = []
    last_valid_data = None
    for _ in range(samples):
        data.motorType = sdk.MotorType.GO_M8010_6
        cmd.motorType = sdk.MotorType.GO_M8010_6
        ok = bool(serial.sendRecv(cmd, data))
        q = float(data.q)
        reason = None
        if not ok:
            reason = "sendRecv returned false (timeout/no reply)"
        elif not bool(data.correct):
            reason = "SDK marked reply incorrect (CRC/frame error)"
        elif int(data.motor_id) != int(motor_id):
            reason = f"reply motor id={int(data.motor_id)}, expected {int(motor_id)}"
        elif int(data.merror) != 0:
            reason = f"motor error={int(data.merror)}"
        elif not math.isfinite(q) or abs(q) > MAX_ABS_ROTOR_RAD:
            reason = f"invalid rotor q={q!r} rad"

        if reason is None:
            vals.append(q)
            last_valid_data = data
        else:
            reject_reasons.append(reason)
        time.sleep(dt)

    required = max(3, math.ceil(samples * MIN_VALID_REPLY_RATIO))
    if len(vals) < required:
        detail = reject_reasons[-1] if reject_reasons else "no valid samples"
        raise RuntimeError(
            f"motor id={motor_id} communication invalid: "
            f"valid replies {len(vals)}/{samples}, required {required}; {detail}"
        )

    spread = max(vals) - min(vals)
    if spread > MAX_SAMPLE_SPREAD_RAD:
        raise RuntimeError(
            f"motor id={motor_id} rotor readings unstable: "
            f"spread={spread:.6f} rad > {MAX_SAMPLE_SPREAD_RAD:.6f} rad"
        )

    return sum(vals) / len(vals), last_valid_data, len(vals), spread


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
        help="Skip the confirmation prompt after manually placing the leg at the fixed calibration pose.",
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
    print("  2. Put the leg at the fixed calibration pose by hand:")
    print("     hip   : no abduction/adduction -> common hip_abduction =   0.00 deg")
    print("     thigh : horizontal              -> common thigh_pitch   = +90.00 deg")
    print("     calf  : fully folded hard stop  -> knee_pitch = -160.59 deg")
    print("                                      and crank angle = +10.00 deg")
    print("     bridge command 0 deg will return to this pose; it is not")
    print("     automatically the common mechanical joint zero.")
    print("  3. Make sure no bridge with --enable-motors is running.")
    print()

    if not args.yes:
        reply = input("Type YES after the leg is at the fixed calibration pose: ").strip()
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
        try:
            q_home, data, valid_count, spread = read_current_rotor(
                sdk=sdk,
                port=cfg["port"],
                motor_id=cfg["id"],
                samples=args.samples,
                dt=args.dt,
            )
        except Exception as exc:
            print(f"  FAILED: {exc}")
            print("Calibration aborted. Existing home file was not changed.")
            return 1
        home[name] = {
            "port": cfg["port"],
            "id": int(cfg["id"]),
            "direction": float(cfg["direction"]),
            "q_home": q_home,
            "gear": gear,
            "motor_type": "GO_M8010_6",
            "calibrated_at": datetime.now().isoformat(timespec="seconds"),
            # 保存 q_home 对应的固定机械姿态，便于审计并防止把 q_home
            # 误解为“所有关节的机械零位”。bridge 的底层控制只使用 q_home；
            # common -> bridge 的偏置由 real_leg_adapter.py 负责。
            "calibration_reference": CALIBRATION_COMMON_REFERENCE[name],
        }
        print(
            f"  q_home={q_home:.6f} rad, valid={valid_count}/{args.samples}, "
            f"spread={spread:.6f} rad, merror={getattr(data, 'merror', 'unknown')}"
        )

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
