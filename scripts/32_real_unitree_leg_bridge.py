#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path


MOTOR_NAMES = ("hip_motor", "thigh_motor", "calf_motor")

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
        "min_deg": -45.0,
        "max_deg": 45.0,
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

KP = 0.2
KD = 0.02
DT = 0.005
RAMP_TIME = 0.35
HOME_FILE = os.path.expanduser("~/motor_home.json")


def clamp_finite(value, lower, upper, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return max(lower, min(upper, value))


def load_home(path=HOME_FILE):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"home file not found: {path}. Run calibration before enabling motors."
        )
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def joint_to_rotor(joint_deg, q_home, gear):
    return q_home + math.radians(joint_deg) * gear


def rotor_to_joint(rotor_rad, q_home, gear):
    return math.degrees((rotor_rad - q_home) / gear)


class MotorRuntime:
    def __init__(self, cfg, sdk):
        self.cfg = cfg
        self.sdk = sdk
        self.serial = sdk.SerialPort(cfg["port"])
        self.cmd = sdk.MotorCmd()
        self.data = sdk.MotorData()
        self.q_home = None

    def init_cmd(self):
        self.cmd.motorType = self.sdk.MotorType.GO_M8010_6
        self.cmd.mode = self.sdk.queryMotorMode(
            self.sdk.MotorType.GO_M8010_6,
            self.sdk.MotorMode.FOC,
        )
        self.cmd.id = self.cfg["id"]

    def read_current_rotor(self, n=10):
        self.init_cmd()
        self.cmd.q = 0.0
        self.cmd.dq = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.01
        self.cmd.tau = 0.0

        vals = []
        for _ in range(n):
            self.data.motorType = self.sdk.MotorType.GO_M8010_6
            self.cmd.motorType = self.sdk.MotorType.GO_M8010_6
            self.serial.sendRecv(self.cmd, self.data)
            vals.append(self.data.q)
            time.sleep(0.01)

        return sum(vals) / len(vals)

    def send_motor(self, q, dq, kp, kd, tau):
        self.init_cmd()
        self.data.motorType = self.sdk.MotorType.GO_M8010_6
        self.cmd.motorType = self.sdk.MotorType.GO_M8010_6
        self.cmd.q = q
        self.cmd.dq = dq
        self.cmd.kp = kp
        self.cmd.kd = kd
        self.cmd.tau = tau
        self.serial.sendRecv(self.cmd, self.data)


class RealUnitreeLegBridge:
    def __init__(self, motors_cfg, enable_motors=False, sdk_path=None):
        self.motors_cfg = motors_cfg
        self.enable_motors = enable_motors
        self.sdk_path = sdk_path

        self.lock = threading.Lock()
        self.running = True
        self.motors_ready = False
        self.gear = None
        self.sdk = None
        self.runtime = {}

        self.target_deg = {name: 0.0 for name in MOTOR_NAMES}
        self.current_deg = {name: 0.0 for name in MOTOR_NAMES}
        self.last_error = ""

    def import_sdk(self):
        if self.sdk_path:
            sys.path.insert(0, str(Path(self.sdk_path).expanduser().resolve()))

        import unitree_actuator_sdk as sdk

        return sdk

    def find_home_entry(self, home, motor_name, cfg):
        for key, value in home.items():
            if value.get("port") == cfg["port"] and int(value.get("id")) == int(cfg["id"]):
                return value
        raise KeyError(
            f"cannot find home for {motor_name}: port={cfg['port']} id={cfg['id']}"
        )

    def start(self):
        if not self.enable_motors:
            self.motors_ready = True
            print("[DRY-RUN] motors disabled. HTTP commands will only update targets.")
            return

        self.sdk = self.import_sdk()
        home = load_home()
        self.gear = self.sdk.queryGearRatio(self.sdk.MotorType.GO_M8010_6)

        print(f"gear ratio: {self.gear:.3f}")

        for name in MOTOR_NAMES:
            cfg = self.motors_cfg[name]
            rt = MotorRuntime(cfg, self.sdk)
            entry = self.find_home_entry(home, name, cfg)
            rt.q_home = float(entry["q_home"])
            self.runtime[name] = rt
            print(
                f"{name}: port={cfg['port']} id={cfg['id']} "
                f"dir={cfg['direction']} home={rt.q_home:.6f} rad"
            )

        print("reading current motor positions...")
        for name in MOTOR_NAMES:
            cfg = self.motors_cfg[name]
            rt = self.runtime[name]
            q_now = rt.read_current_rotor()
            joint_deg = rotor_to_joint(q_now, rt.q_home, self.gear) * cfg["direction"]

            with self.lock:
                self.current_deg[name] = joint_deg
                self.target_deg[name] = joint_deg

            print(f"{name}: current={joint_deg:.2f} deg")

        self.motors_ready = True
        thread = threading.Thread(target=self.control_loop, daemon=True)
        thread.start()

    def control_loop(self):
        prev_target = dict(self.target_deg)
        ramp_q0 = {}
        ramp_t0 = {}

        print("[REAL] control loop started")

        try:
            while self.running:
                now = time.time()

                for name in MOTOR_NAMES:
                    cfg = self.motors_cfg[name]
                    rt = self.runtime[name]

                    with self.lock:
                        target_joint = self.target_deg[name] * cfg["direction"]

                    if abs(target_joint - prev_target.get(name, 999.0)) > 0.01:
                        prev_target[name] = target_joint
                        ramp_q0[name] = rt.data.q
                        ramp_t0[name] = now

                    elapsed = now - ramp_t0.get(name, now)
                    ratio = min(max(elapsed / RAMP_TIME, 0.0), 1.0)
                    ease = 0.5 - 0.5 * math.cos(math.pi * ratio)

                    q_target = joint_to_rotor(target_joint, rt.q_home, self.gear)
                    q_start = ramp_q0.get(name, q_target)
                    q_cmd = q_start + (q_target - q_start) * ease
                    dq_ff = (q_target - q_start) / RAMP_TIME if ratio < 1.0 else 0.0

                    rt.send_motor(q_cmd, dq_ff, KP, KD, 0.0)

                    joint_now = rotor_to_joint(rt.data.q, rt.q_home, self.gear)
                    with self.lock:
                        self.current_deg[name] = joint_now * cfg["direction"]

                    if rt.data.merror != 0:
                        print(f"WARNING {name} merror={rt.data.merror}")

                time.sleep(DT)

        except Exception as exc:
            self.last_error = str(exc)
            print(f"control loop error: {exc}")
        finally:
            self.safe_stop_all()
            self.motors_ready = False

    def set_targets(self, body):
        new_targets = {}

        for name in MOTOR_NAMES:
            cfg = self.motors_cfg[name]
            new_targets[name] = clamp_finite(
                body[name],
                cfg["min_deg"],
                cfg["max_deg"],
                name,
            )

        with self.lock:
            self.target_deg.update(new_targets)

        return new_targets

    def safe_stop_all(self):
        if not self.enable_motors or not self.runtime:
            return

        print("[safe_stop_all] fading kp/kd")
        for step in range(200):
            fade = 1.0 - step / 200.0
            for name, rt in self.runtime.items():
                cfg = self.motors_cfg[name]
                rt.send_motor(rt.data.q, 0.0, KP * fade, KD * fade, 0.0)
            time.sleep(DT)

    def status(self):
        with self.lock:
            return {
                "ok": True,
                "enable_motors": self.enable_motors,
                "motors_ready": self.motors_ready,
                "target_deg": dict(self.target_deg),
                "current_deg": dict(self.current_deg),
                "last_error": self.last_error,
            }

    def stop(self):
        self.running = False


def make_handler(bridge):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send_json(self, data, code=200):
            body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/status":
                self.send_json(bridge.status())
                return

            self.send_json({"ok": False, "error": "not found"}, 404)

        def do_POST(self):
            if self.path == "/stop":
                bridge.stop()
                self.send_json({"ok": True, "message": "stopping"})
                return

            if self.path != "/set_motor_commands":
                self.send_json({"ok": False, "error": "not found"}, 404)
                return

            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError("JSON body must be an object")

                for name in MOTOR_NAMES:
                    if name not in body:
                        raise ValueError(f"missing field: {name}")

                accepted = bridge.set_targets(body)

            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, 400)
                return

            print(
                "target "
                f"hip_motor={accepted['hip_motor']:7.2f}deg "
                f"thigh_motor={accepted['thigh_motor']:7.2f}deg "
                f"calf_motor={accepted['calf_motor']:7.2f}deg"
            )

            self.send_json(
                {
                    "ok": True,
                    "enable_motors": bridge.enable_motors,
                    "target": accepted,
                }
            )

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument(
        "--sdk-path",
        default=None,
        help="Folder containing unitree_actuator_sdk.py or compiled SDK module.",
    )
    args = parser.parse_args()

    bridge = RealUnitreeLegBridge(
        motors_cfg=DEFAULT_MOTORS,
        enable_motors=args.enable_motors,
        sdk_path=args.sdk_path,
    )

    print("real Unitree leg bridge")
    print(f"enable_motors: {args.enable_motors}")
    print(f"POST http://{args.host}:{args.port}/set_motor_commands")
    print(f"GET  http://{args.host}:{args.port}/status")
    print(f"POST http://{args.host}:{args.port}/stop")
    print()

    try:
        bridge.start()
    except Exception as exc:
        print(f"bridge startup failed: {exc}")
        sys.exit(1)

    server = HTTPServer((args.host, args.port), make_handler(bridge))

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        bridge.stop()
        server.server_close()
        bridge.safe_stop_all()
        print("bridge closed")


if __name__ == "__main__":
    main()
