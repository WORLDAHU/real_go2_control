"""
单腿实机命令流式发送脚本。

用途：
- 跑 MuJoCo 单腿控制器。
- 从 info["q_des"] 取得目标仿真关节角。
- 通过 RealLegCommandAdapter 转成实机 bridge 命令。
- 可以选择只打印，也可以通过 HTTP 发给 bridge。

模式：
  --mode print
      只在终端打印，不发送 HTTP。

  --mode http
      向指定 URL 发送 JSON 命令。

注意：
这个脚本仍然不应该直接控制电机。
真正的串口、电机 ID、dir、零点加载，应放在更底层的 motor bridge 里。
"""
import argparse
import json
import sys
import time
import urllib.request
import urllib.parse
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from single_leg_slider_controller import SingleLegSliderController
from real_leg_adapter import RealLegCommandAdapter


MJCF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"


def post_json(url, body):
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=0.5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def bridge_stop_url(command_url):
    parsed = urllib.parse.urlsplit(command_url)
    return urllib.parse.urlunsplit(
        (parsed.scheme, parsed.netloc, "/stop", "", "")
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["print", "http"], default="print")
    parser.add_argument("--url", default="http://127.0.0.1:8765/set_motor_commands")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive")
    if args.duration is not None and args.duration <= 0.0:
        raise ValueError("--duration must be positive")

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    controller = SingleLegSliderController(model)
    controller.initialize_targets(data)

    adapter = RealLegCommandAdapter()

    dt = 1.0 / args.rate
    end_time = args.duration if args.duration is not None else controller.total_time

    print("stream real-leg commands")
    print("mode:", args.mode)
    print("rate:", args.rate, "Hz")
    print("end_time:", end_time, "s")
    if args.mode == "http":
        print("url:", args.url)
    print("Ctrl+C will request bridge /stop in HTTP mode")
    print()

    next_send_wall = time.time()

    try:
        while data.time < end_time:
            ctrl, info = controller.compute_control(data)
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

            now = time.time()
            if now < next_send_wall:
                time.sleep(next_send_wall - now)

            cmd = adapter.q_des_to_command(info["q_des"])
            bridge_cmd = cmd["bridge_cmd_deg"]

            body = {
                "hip_motor": bridge_cmd["hip_motor"],
                "thigh_motor": bridge_cmd["thigh_motor"],
                "calf_motor": bridge_cmd["calf_motor"],
            }

            if args.mode == "http":
                resp = post_json(args.url, body)
                ok = resp.get("ok", False)
                if not ok:
                    raise RuntimeError(f"bridge rejected command: {resp}")
            else:
                ok = True

            print(
                f"t={data.time:5.2f}s "
                f"phase={info['phase']:16s} "
                f"ok={str(ok):5s} "
                f"hip_motor={body['hip_motor']:7.2f} "
                f"thigh_motor={body['thigh_motor']:7.2f} "
                f"calf_motor={body['calf_motor']:7.2f}"
            )

            next_send_wall += dt

    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        if args.mode == "http":
            stop_url = bridge_stop_url(args.url)
            try:
                response = post_json(stop_url, {})
                confirmed = bool(
                    response.get("ok")
                    and response.get("stop_status", {}).get(
                        "stop_reply_all_valid", False
                    )
                )
                print(f"bridge stop requested: confirmed={confirmed}")
                if not confirmed:
                    print("WARNING: stop replies were not fully confirmed; cut motor power.")
            except Exception as exc:
                print(f"WARNING: bridge /stop failed: {exc}")
                print("Cut motor power and verify no bridge process remains.")


if __name__ == "__main__":
    main()
