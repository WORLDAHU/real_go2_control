#!/usr/bin/env python3
import argparse
import math
import sys
import time
from pathlib import Path


DEFAULT_MOTORS = {
    "hip_motor": {"label": "hip", "port": "/dev/ttyUSB2", "id": 2},
    "thigh_motor": {"label": "thigh", "port": "/dev/ttyUSB1", "id": 0},
    "calf_motor": {"label": "calf", "port": "/dev/ttyUSB0", "id": 1},
}
TWO_PI = 2.0 * math.pi


def unwrap_near(angle_rad, reference_rad):
    return float(angle_rad) + round(
        (float(reference_rad) - float(angle_rad)) / TWO_PI
    ) * TWO_PI


def import_sdk(sdk_path):
    if sdk_path:
        sys.path.insert(0, str(Path(sdk_path).expanduser().resolve()))
    import unitree_actuator_sdk as sdk
    return sdk


def stop_mode_value(sdk):
    motor_mode = getattr(sdk, "MotorMode", None)
    stop_enum = getattr(motor_mode, "STOP", None) if motor_mode is not None else None
    if stop_enum is None:
        stop_enum = getattr(motor_mode, "Stop", None) if motor_mode is not None else None
    if stop_enum is None:
        stop_enum = getattr(motor_mode, "BRAKE", None) if motor_mode is not None else None

    if stop_enum is not None:
        try:
            return sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, stop_enum)
        except Exception:
            pass

    # The GO-M8010-6 manual documents mode 0 as stop.
    return 0


def foc_mode_value(sdk):
    return sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)


def send_cmd(sdk, serial, cmd, data, motor_id, mode, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0, allow_motor_fault=False):
    cmd.motorType = sdk.MotorType.GO_M8010_6
    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.mode = mode
    cmd.id = int(motor_id)
    cmd.q = q
    cmd.dq = dq
    cmd.kp = kp
    cmd.kd = kd
    cmd.tau = tau
    ok = bool(serial.sendRecv(cmd, data))
    if not ok:
        raise RuntimeError("sendRecv timeout/no reply")
    if not bool(data.correct):
        raise RuntimeError("invalid CRC/frame")
    if int(data.motor_id) != int(motor_id):
        raise RuntimeError(
            f"reply id={int(data.motor_id)}, expected {int(motor_id)}"
        )
    if not math.isfinite(float(data.q)) or abs(float(data.q)) > 10000.0:
        raise RuntimeError(f"invalid rotor q={float(data.q)!r}")
    if not allow_motor_fault and int(data.merror) != 0:
        raise RuntimeError(f"motor fault merror={int(data.merror)}")
    return int(data.merror)


def send_stop_best_effort(sdk, serial, cmd, data, motor_id, stop_mode):
    try:
        send_cmd(
            sdk, serial, cmd, data, motor_id, mode=stop_mode,
            q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0,
            allow_motor_fault=True,
        )
        return True, int(data.merror)
    except Exception as exc:
        print(f"  stop reply invalid: {exc}")
        return False, None


def read_current_q(sdk, serial, cmd, data, motor_id, samples, dt):
    foc_mode = foc_mode_value(sdk)
    vals = []
    err = 0
    for _ in range(samples):
        err = send_cmd(
            sdk,
            serial,
            cmd,
            data,
            motor_id,
            mode=foc_mode,
            q=0.0,
            dq=0.0,
            kp=0.0,
            kd=0.0,
            tau=0.0,
        )
        raw_q = float(data.q)
        vals.append(raw_q if not vals else unwrap_near(raw_q, vals[-1]))
        time.sleep(dt)
    spread = max(vals) - min(vals)
    if spread > 0.10:
        raise RuntimeError(f"rotor readings unstable: spread={spread:.6f} rad")
    return sum(vals) / len(vals), err


def release_motor(sdk, cfg, fade_steps, stop_steps, dt):
    serial = sdk.SerialPort(cfg["port"])
    cmd = sdk.MotorCmd()
    data = sdk.MotorData()
    foc_mode = foc_mode_value(sdk)
    stop_mode = stop_mode_value(sdk)

    print(
        f"release {cfg['label']}: port={cfg['port']} id={cfg['id']} "
        f"stop_mode={stop_mode}"
    )

    position_valid = False
    try:
        q_hold, err = read_current_q(
            sdk=sdk, serial=serial, cmd=cmd, data=data,
            motor_id=cfg["id"], samples=10, dt=dt,
        )
        position_valid = True
        print(f"  current rotor q={q_hold:+.6f} rad, merror={err}")

        # Only a fully validated position may be used for the active fade.
        for i in range(fade_steps):
            fade = 1.0 - i / max(fade_steps, 1)
            send_cmd(
                sdk, serial, cmd, data, cfg["id"], mode=foc_mode,
                q=q_hold, dq=0.0, kp=0.05 * fade, kd=0.01 * fade, tau=0.0,
            )
            time.sleep(dt)
    except Exception as exc:
        print(f"  active fade skipped/aborted: {exc}")

    # Then explicitly switch the motor command to stop mode.
    valid = 0
    zero_error = 0
    for _ in range(stop_steps):
        ok, err = send_stop_best_effort(
            sdk, serial, cmd, data, cfg["id"], stop_mode
        )
        valid += int(ok)
        zero_error += int(ok and err == 0)
        time.sleep(dt)

    print(
        f"  stop replies valid={valid}/{stop_steps}, "
        f"zero_error={zero_error}/{stop_steps}"
    )
    if valid != stop_steps:
        print("  WARNING: software could not confirm every stop reply.")
    return position_valid, valid == stop_steps and zero_error == stop_steps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    parser.add_argument(
        "--motor",
        choices=["all"] + sorted(DEFAULT_MOTORS),
        default="all",
        help="Which motor to release. Default: all.",
    )
    parser.add_argument("--fade-steps", type=int, default=80)
    parser.add_argument("--stop-steps", type=int, default=20)
    parser.add_argument("--dt", type=float, default=0.01)
    args = parser.parse_args()
    if args.fade_steps < 0 or args.stop_steps <= 0 or not math.isfinite(args.dt) or args.dt <= 0.0:
        print("fade-steps must be >=0, stop-steps >0, and dt finite/positive.")
        return 1

    sdk = import_sdk(args.sdk_path)
    names = sorted(DEFAULT_MOTORS) if args.motor == "all" else [args.motor]

    print("Release Unitree leg motor(s)")
    print("This sends zero-stiffness FOC commands, then mode=0 stop commands.")
    print()

    all_confirmed = True
    for name in names:
        _, stop_confirmed = release_motor(
            sdk=sdk,
            cfg=DEFAULT_MOTORS[name],
            fade_steps=args.fade_steps,
            stop_steps=args.stop_steps,
            dt=args.dt,
        )
        all_confirmed = all_confirmed and stop_confirmed

    print("Stop frames sent. Physical motor power removal cannot be verified in software.")
    return 0 if all_confirmed else 1


if __name__ == "__main__":
    raise SystemExit(main())
