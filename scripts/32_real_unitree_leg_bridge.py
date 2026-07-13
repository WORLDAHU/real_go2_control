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
# 复用唯一的小腿四连杆模型，把 bridge 电机角转换回 common 机械角。
# 这样 status、RL 和实机映射都使用同一套几何关系。
SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))



from real_leg_adapter import RealLegCommandAdapter

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
        # bridge 坐标以“上电标定时的大腿水平”为 0 deg。
        # common thigh 的机械安全范围仍为 [-30, +90] deg，标定姿态是
        # common +90 deg，因此 bridge 范围为 [-30-90, +90-90] = [-120, 0]。
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

KP = 0.2
KD = 0.02
DT = 0.02
RAMP_TIME = 1.0
MAX_COMMAND_SPEED_DEG_S = 20.0
# 普通 HTTP 命令会同时受 ramp_time 和 MAX_COMMAND_SPEED_DEG_S 限制；即使调用者
# 请求 0.05 秒，大角度阶跃也会自动延长。启动归位另用更慢的 home_ramp_time。
HOME_RAMP_TIME = 3.0
HOME_FILE = os.path.expanduser("~/motor_home.json")
MAX_ABS_Q_HOME_RAD = 10000.0

# 这里只能校验文件声明的元数据，不能从非绝对编码器判断人工摆放的真实姿态。
# 小腿 common 角由当前四连杆几何和曲柄标定角自动计算，不再重复写死。
_CALIBRATION_MODEL = RealLegCommandAdapter()
EXPECTED_CALIBRATION_METADATA = {
    "hip_motor": {"common_deg": 0.0},
    "thigh_motor": {"common_deg": 90.0},
    "calf_motor": {
        "common_deg": _CALIBRATION_MODEL.fourbar.knee_pitch_home_deg,
        "crank_deg": _CALIBRATION_MODEL.fourbar.crank_home_deg,
    },
}


def clamp_finite(value, lower, upper, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if value < lower or value > upper:
        raise ValueError(
            f"{name}={value:+.2f} deg outside safe range "
            f"[{lower:+.2f}, {upper:+.2f}] deg"
        )
    return value


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


def validate_calibration_metadata(entry, motor_name):
    """
    检查文件声明的标定元数据是否与当前代码约定一致。

    这不能验证标定时腿是否真的摆在对应机械姿态；该事实只能由操作者或额外
    的绝对传感器确认。这里仅防止旧格式、不同几何参数或不同标定约定的文件
    被当前代码误用。
    """
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


def validate_home_numeric(entry, motor_name, expected_gear):
    """拒绝超时/坏帧产生的垃圾 q_home 和不匹配的减速比。"""
    q_home = float(entry.get("q_home", float("nan")))
    if not math.isfinite(q_home) or abs(q_home) > MAX_ABS_Q_HOME_RAD:
        raise ValueError(f"{motor_name} invalid q_home={q_home!r}")

    stored_gear = float(entry.get("gear", float("nan")))
    if not math.isfinite(stored_gear) or stored_gear <= 0.0:
        raise ValueError(f"{motor_name} invalid gear={stored_gear!r}")
    if abs(stored_gear - float(expected_gear)) > 1e-3:
        raise ValueError(
            f"{motor_name} home gear={stored_gear:.6f}, "
            f"current SDK gear={float(expected_gear):.6f}"
        )


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
        ok = bool(self.serial.sendRecv(self.cmd, self.data))
        if not ok:
            raise RuntimeError(
                f"{self.cfg['label']} sendRecv timeout/no reply"
            )
        if not bool(self.data.correct):
            raise RuntimeError(
                f"{self.cfg['label']} invalid CRC/frame"
            )
        if int(self.data.motor_id) != int(self.cfg["id"]):
            raise RuntimeError(
                f"{self.cfg['label']} reply id={int(self.data.motor_id)}, "
                f"expected {int(self.cfg['id'])}"
            )
        if not math.isfinite(float(self.data.q)):
            raise RuntimeError(f"{self.cfg['label']} returned invalid q")
        if int(self.data.merror) != 0:
            raise RuntimeError(
                f"{self.cfg['label']} motor fault merror={int(self.data.merror)}"
            )

    def send_stop(self):
        self.init_cmd()
        stop_mode = getattr(self.sdk.MotorMode, "STOP", None)
        if stop_mode is not None:
            try:
                self.cmd.mode = self.sdk.queryMotorMode(
                    self.sdk.MotorType.GO_M8010_6,
                    stop_mode,
                )
            except Exception:
                self.cmd.mode = 0
        else:
            self.cmd.mode = 0

        self.data.motorType = self.sdk.MotorType.GO_M8010_6
        self.cmd.motorType = self.sdk.MotorType.GO_M8010_6
        self.cmd.q = 0.0
        self.cmd.dq = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.0
        self.cmd.tau = 0.0
        self.serial.sendRecv(self.cmd, self.data)


class RealUnitreeLegBridge:
    def __init__(
        self,
        motors_cfg,
        enable_motors=False,
        sdk_path=None,
        kp=KP,
        kd=KD,
        dt=DT,
        ramp_time=RAMP_TIME,
        home_ramp_time=HOME_RAMP_TIME,
    ):
        self.motors_cfg = motors_cfg
        self.enable_motors = enable_motors
        self.sdk_path = sdk_path
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.ramp_time = ramp_time
        self.home_ramp_time = home_ramp_time

        self.lock = threading.Lock()
        self.running = True
        self.stopped = False
        self.stopping = False
        self.motors_ready = False
        self.gear = None
        self.sdk = None
        self.runtime = {}

        # 用于 bridge_cmd_deg -> common_joint_deg 的正向换算。
        #
        # bridge 坐标：
        #   hip=0   -> common hip=0
        #   thigh=0 -> common thigh=+90（大腿水平标定姿态）
        #   calf=0  -> common knee 由当前四连杆几何自动计算（小腿收缩限位）
        self.angle_adapter = RealLegCommandAdapter()

        self.target_deg = {name: 0.0 for name in MOTOR_NAMES}
        # 每个目标可拥有不同的缓动时间。正常 HTTP / 38 目标使用 ramp_time；
        # 启动回固定标定姿态使用更慢的 home_ramp_time。
        self.target_ramp_time = {name: ramp_time for name in MOTOR_NAMES}
        self.current_deg = {name: 0.0 for name in MOTOR_NAMES}
        self.last_error = ""
        self.last_accepted_ramp_time = ramp_time

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
        try:
            home = load_home()
        except FileNotFoundError as exc:
            print("[startup] 未找到上电标定文件 ~/motor_home.json。")
            print("[startup] 请先释放电机、摆到固定标定姿态，然后运行：")
            print("  /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \\")
            print("    scripts/33_calibrate_motor_home.py \\")
            print("    --sdk-path /home/claww/unitree_actuator_sdk/lib")
            raise RuntimeError("cannot enable bridge without a valid motor home") from exc
        self.gear = self.sdk.queryGearRatio(self.sdk.MotorType.GO_M8010_6)

        print(f"gear ratio: {self.gear:.3f}")

        try:
            for name in MOTOR_NAMES:
                cfg = self.motors_cfg[name]
                rt = MotorRuntime(cfg, self.sdk)
                entry = self.find_home_entry(home, name, cfg)
                validate_calibration_metadata(entry, name)
                validate_home_numeric(entry, name, self.gear)
                rt.q_home = float(entry["q_home"])
                self.runtime[name] = rt
                reference = entry["calibration_reference"]
                print(
                    f"{name}: port={cfg['port']} id={cfg['id']} "
                    f"dir={cfg['direction']} home={rt.q_home:.6f} rad"
                )
                print(
                    f"  calibrated_at={entry.get('calibrated_at', 'unknown')} "
                    f"common={reference.get('common_deg')} deg "
                    f"({reference.get('description', '')})"
                )
        except (KeyError, ValueError) as exc:
            print(f"[startup] 标定文件元数据或数值无效：{exc}")
            print("[startup] 此检查不能验证人工摆放的真实机械姿态。")
            print("[startup] 请确认固定姿态后重新运行 scripts/33_calibrate_motor_home.py。")
            raise RuntimeError("invalid motor home calibration") from exc

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

        print()
        print("固定标定姿态将作为启动归位目标：")
        print("  hip_motor=0 deg   ：髋无内外摆动")
        print("  thigh_motor=0 deg ：大腿水平（common thigh=+90 deg）")
        print("  calf_motor=0 deg  ：小腿完全收缩（crank=10 deg）")
        print(
            f"本次启动归位使用 {self.home_ramp_time:.2f} 秒缓动；普通 HTTP 目标 "
            f"仍使用 {self.ramp_time:.2f} 秒缓动。"
        )
        reply = input(
            "Type YES to slowly move to the fixed calibration pose; "
            "otherwise bridge will stop and you should recalibrate: "
        ).strip()
        if reply != "YES":
            print("启动已取消：未向标定姿态归位。请释放/摆腿后重新运行 33 标定。")
            self.stop_all_immediately()
            raise RuntimeError("startup cancelled; recalibration required")

        # 首次控制循环会从刚才读取的实际 q 开始，以 home_ramp_time 缓动到
        # bridge 0/0/0，也就是本次 q_home 对应的固定标定姿态。
        with self.lock:
            for name in MOTOR_NAMES:
                self.target_deg[name] = 0.0
                self.target_ramp_time[name] = self.home_ramp_time

        print("[startup] 已确认：正在慢速回到固定标定姿态。")
        self.motors_ready = True
        thread = threading.Thread(target=self.control_loop, daemon=True)
        thread.start()

    def control_loop(self):
        # 置空使启动确认后的 0/0/0 被识别为新目标；q_start 是启动时读取到
        # 的实际转子位置，而不是 q_home。
        prev_target = {}
        ramp_q0 = {}
        ramp_t0 = {}
        ramp_duration = {}

        print("[REAL] control loop started")

        try:
            while self.running:
                now = time.time()

                for name in MOTOR_NAMES:
                    cfg = self.motors_cfg[name]
                    rt = self.runtime[name]

                    with self.lock:
                        target_joint = self.target_deg[name] * cfg["direction"]
                        requested_ramp_time = self.target_ramp_time[name]

                    if abs(target_joint - prev_target.get(name, 999.0)) > 0.01:
                        prev_target[name] = target_joint
                        ramp_q0[name] = rt.data.q
                        ramp_t0[name] = now
                        ramp_duration[name] = requested_ramp_time

                    elapsed = now - ramp_t0.get(name, now)
                    duration = ramp_duration.get(name, self.ramp_time)
                    ratio = min(max(elapsed / duration, 0.0), 1.0)
                    ease = 0.5 - 0.5 * math.cos(math.pi * ratio)

                    q_target = joint_to_rotor(target_joint, rt.q_home, self.gear)
                    q_start = ramp_q0.get(name, q_target)
                    q_cmd = q_start + (q_target - q_start) * ease
                    dq_ff = (
                        (q_target - q_start)
                        * 0.5
                        * math.pi
                        / duration
                        * math.sin(math.pi * ratio)
                        if ratio < 1.0
                        else 0.0
                    )

                    rt.send_motor(q_cmd, dq_ff, self.kp, self.kd, 0.0)

                    joint_now = rotor_to_joint(rt.data.q, rt.q_home, self.gear)
                    with self.lock:
                        self.current_deg[name] = joint_now * cfg["direction"]

                time.sleep(self.dt)

        except Exception as exc:
            self.last_error = str(exc)
            print(f"EMERGENCY STOP: {exc}")
            self.stop_all_immediately()
        finally:
            self.motors_ready = False

    def set_targets(self, body, ramp_time=None):
        """
        设置普通 HTTP / 38 / RL 目标。

        未指定 ramp_time 时使用 --ramp-time；无论调用者请求多短的时间，最终
        轨迹都不能超过 MAX_COMMAND_SPEED_DEG_S。启动归位使用独立的
        --home-ramp-time。
        """
        new_targets = {}
        if not self.motors_ready or self.stopped or not self.running:
            raise RuntimeError("bridge motors are not ready or a fault is latched")

        requested_ramp_time = self.ramp_time if ramp_time is None else float(ramp_time)
        if not math.isfinite(requested_ramp_time) or requested_ramp_time <= 0.0:
            raise ValueError("ramp_time must be positive")

        for name in MOTOR_NAMES:
            cfg = self.motors_cfg[name]
            new_targets[name] = clamp_finite(
                body[name],
                cfg["min_deg"],
                cfg["max_deg"],
                name,
            )

        with self.lock:
            required_ramp_time = max(
                abs(new_targets[name] - self.current_deg[name])
                / MAX_COMMAND_SPEED_DEG_S
                for name in MOTOR_NAMES
            )
            effective_ramp_time = max(requested_ramp_time, required_ramp_time)
            self.target_deg.update(new_targets)
            for name in MOTOR_NAMES:
                self.target_ramp_time[name] = effective_ramp_time
            self.last_accepted_ramp_time = effective_ramp_time

        return new_targets

    def stop_all_immediately(self):
        """启动确认被取消时，仅发送 mode=0，不建立位置保持。"""
        if not self.runtime:
            return
        print("[startup] sending motor stop mode")
        for _ in range(3):
            for rt in self.runtime.values():
                rt.send_stop()
            time.sleep(self.dt)
        self.stopped = True
        self.running = False
        self.motors_ready = False

    def safe_stop_all(self):
        if not self.enable_motors or not self.runtime:
            return
        if self.stopped or self.stopping:
            return

        self.stopping = True

        print("[safe_stop_all] fading kp/kd")
        fade_steps = max(1, int(1.0 / self.dt))
        for step in range(fade_steps):
            fade = 1.0 - step / fade_steps
            for name, rt in self.runtime.items():
                cfg = self.motors_cfg[name]
                rt.send_motor(rt.data.q, 0.0, self.kp * fade, self.kd * fade, 0.0)
            time.sleep(self.dt)

        print("[safe_stop_all] sending motor stop mode")
        for _ in range(20):
            for rt in self.runtime.values():
                rt.send_stop()
            time.sleep(self.dt)

        self.stopped = True
        self.stopping = False

    def bridge_to_common_deg(self, bridge_deg):
        """
        将 bridge 电机坐标转换为固定 common 机械关节坐标。

        bridge 坐标是“相对本次上电标定姿态”的执行器角；
        common 坐标是仿真、RL、运动学讨论使用的固定机械角。

        对髋：
            common hip = bridge hip

        对大腿：
            标定时大腿水平，即 common thigh=+90 deg。
            因此：
                common thigh = bridge thigh + 90 deg

        对小腿：
            不能线性加偏置，必须经过四连杆正解：
                calf bridge command
                    -> crank
                    -> rocker
                    -> common knee_pitch
        """
        return {
            "hip_abduction": float(bridge_deg["hip_motor"]),
            "thigh_pitch": float(bridge_deg["thigh_motor"]) + 90.0,
            "knee_pitch": self.angle_adapter.calf_motor_to_knee_pitch(
                float(bridge_deg["calf_motor"])
            ),
        }

    @staticmethod
    def subtract_angles(target, current):
        """
        计算 target - current。

        bridge 坐标和当前 common 工作范围都不会跨越 ±180 度，因此这里直接相减。
        正误差表示“实际值还没有到目标值”。
        """
        return {
            name: float(target[name]) - float(current[name])
            for name in target
        }

    def status(self):
        """
        HTTP GET /status 返回四类信息：

        1. target_deg / current_deg
           bridge 电机坐标；保留旧字段，兼容 38 等现有脚本。

        2. target_common_deg / current_common_deg
           换算后的固定机械关节坐标，供人工检查、日志和后续 RL 使用。

        3. tracking_error_bridge_deg / tracking_error_common_deg
           目标减实际。若误差长期很大，先检查是否有机械阻挡、刚度不足、
           通信超时，或是否存在另一个程序同时控制同一电机。

        4. common_mapping_error
           四连杆正解失败时给出原因；正常情况下应为字符串空值。
        """
        with self.lock:
            target_bridge = dict(self.target_deg)
            current_bridge = dict(self.current_deg)
            last_error = self.last_error

        common_mapping_error = ""
        try:
            target_common = self.bridge_to_common_deg(target_bridge)
            current_common = self.bridge_to_common_deg(current_bridge)
            tracking_error_common = self.subtract_angles(
                target_common,
                current_common,
            )
        except Exception as exc:
            # 即使四连杆映射异常，也保留原始 bridge 坐标，方便定位问题。
            target_common = {}
            current_common = {}
            tracking_error_common = {}
            common_mapping_error = str(exc)

        return {
            "ok": True,
            "enable_motors": self.enable_motors,
            "motors_ready": self.motors_ready,

            # 原有 bridge 坐标：相对上电标定姿态的电机命令/读数。
            "target_deg": target_bridge,
            "current_deg": current_bridge,
            "tracking_error_bridge_deg": self.subtract_angles(
                target_bridge,
                current_bridge,
            ),

            # 新增 common 机械坐标：用于和仿真/RL 直接比较。
            "target_common_deg": target_common,
            "current_common_deg": current_common,
            "tracking_error_common_deg": tracking_error_common,
            "common_mapping_error": common_mapping_error,

            "last_error": last_error,
            "kp": self.kp,
            "kd": self.kd,
            "dt": self.dt,
            "ramp_time": self.ramp_time,
            "last_accepted_ramp_time": self.last_accepted_ramp_time,
            "max_command_speed_deg_s": MAX_COMMAND_SPEED_DEG_S,
            "home_ramp_time": self.home_ramp_time,
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

                accepted = bridge.set_targets(body, ramp_time=body.get("ramp_time"))

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
                    "ramp_time": bridge.last_accepted_ramp_time,
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
    parser.add_argument("--kp", type=float, default=KP)
    parser.add_argument("--kd", type=float, default=KD)
    parser.add_argument(
        "--dt",
        type=float,
        default=DT,
        help="Bridge control loop period in seconds. Use 0.02 for stable 3-USB tests.",
    )
    parser.add_argument(
        "--ramp-time",
        type=float,
        default=RAMP_TIME,
        help="Requested target smoothing time; the bridge also enforces a conservative speed limit.",
    )
    parser.add_argument(
        "--home-ramp-time",
        type=float,
        default=HOME_RAMP_TIME,
        help="Seconds used only for confirmed startup motion to bridge 0/0/0.",
    )
    args = parser.parse_args()

    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.ramp_time <= 0.0:
        raise ValueError("--ramp-time must be positive")
    if args.home_ramp_time <= 0.0:
        raise ValueError("--home-ramp-time must be positive")

    bridge = RealUnitreeLegBridge(
        motors_cfg=DEFAULT_MOTORS,
        enable_motors=args.enable_motors,
        sdk_path=args.sdk_path,
        kp=args.kp,
        kd=args.kd,
        dt=args.dt,
        ramp_time=args.ramp_time,
        home_ramp_time=args.home_ramp_time,
    )

    print("real Unitree leg bridge")
    print(f"enable_motors: {args.enable_motors}")
    print(
        f"kp={args.kp:.3f} kd={args.kd:.3f} dt={args.dt:.3f}s "
        f"ramp_time={args.ramp_time:.3f}s home_ramp_time={args.home_ramp_time:.3f}s"
    )
    print(f"POST http://{args.host}:{args.port}/set_motor_commands")
    print(f"GET  http://{args.host}:{args.port}/status")
    print(f"POST http://{args.host}:{args.port}/stop")
    print()

    # 先占用 HTTP 端口，再打开电机串口、询问 YES、启动归位控制。
    # 若已有 bridge 占用 8765，必须在任何电机命令之前失败；否则两个 bridge
    # 可能同时向同一个 USB/RS485 电机发送命令。
    try:
        server = HTTPServer((args.host, args.port), make_handler(bridge))
    except OSError as exc:
        print(f"bridge startup refused: cannot bind {args.host}:{args.port}: {exc}")
        print("Another bridge may already be running. Stop it with:")
        print(f"  curl -X POST http://{args.host}:{args.port}/stop")
        sys.exit(1)

    try:
        bridge.start()
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped by user")
    except Exception as exc:
        print(f"bridge startup/runtime failed: {exc}")
    finally:
        bridge.stop()
        server.server_close()
        bridge.safe_stop_all()
        print("bridge closed")


if __name__ == "__main__":
    main()
