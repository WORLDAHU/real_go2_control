#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path


DEFAULT_MOTORS = {
    "hip_motor": {"label": "hip", "port": "/dev/ttyUSB2", "id": 2},
    "thigh_motor": {"label": "thigh", "port": "/dev/ttyUSB1", "id": 0},
    "calf_motor": {"label": "calf", "port": "/dev/ttyUSB0", "id": 1},
}


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


def send_cmd(sdk, serial, cmd, data, motor_id, mode, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0):
    cmd.motorType = sdk.MotorType.GO_M8010_6
    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.mode = mode
    cmd.id = int(motor_id)
    cmd.q = q
    cmd.dq = dq
    cmd.kp = kp
    cmd.kd = kd
    cmd.tau = tau
    serial.sendRecv(cmd, data)
    return int(data.merror)


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
        vals.append(float(data.q))
        time.sleep(dt)
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

    q_hold, err = read_current_q(
        sdk=sdk,
        serial=serial,
        cmd=cmd,
        data=data,
        motor_id=cfg["id"],
        samples=10,
        dt=dt,
    )
    print(f"  current rotor q={q_hold:+.6f} rad, merror={err}")

    # First remove active stiffness while still communicating in FOC.
    # Hold the current rotor position; do not command q=0 during release.
    for i in range(fade_steps):
        fade = 1.0 - i / max(fade_steps, 1)
        err = send_cmd(
            sdk,
            serial,
            cmd,
            data,
            cfg["id"],
            mode=foc_mode,
            q=q_hold,
            dq=0.0,
            kp=0.05 * fade,
            kd=0.01 * fade,
            tau=0.0,
        )
        time.sleep(dt)

    # Then explicitly switch the motor command to stop mode.
    for _ in range(stop_steps):
        err = send_cmd(
            sdk,
            serial,
            cmd,
            data,
            cfg["id"],
            mode=stop_mode,
            q=0.0,
            dq=0.0,
            kp=0.0,
            kd=0.0,
            tau=0.0,
        )
        time.sleep(dt)

    print(f"  done, merror={err}")


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

    sdk = import_sdk(args.sdk_path)
    names = sorted(DEFAULT_MOTORS) if args.motor == "all" else [args.motor]

    print("Release Unitree leg motor(s)")
    print("This sends zero-stiffness FOC commands, then mode=0 stop commands.")
    print()

    for name in names:
        release_motor(
            sdk=sdk,
            cfg=DEFAULT_MOTORS[name],
            fade_steps=args.fade_steps,
            stop_steps=args.stop_steps,
            dt=args.dt,
        )

    print("All requested motors released.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
