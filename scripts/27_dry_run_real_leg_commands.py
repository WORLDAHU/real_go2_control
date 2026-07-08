"""
单腿实机命令 dry-run 脚本。

用途：
- 跑当前 MuJoCo 单腿控制器。
- 不打开 viewer。
- 不连接真实电机。
- 把控制器输出的 q_des 转成实机角度命令并打印。

这个脚本适合检查：
- q_des 是否合理。
- sim/common/bridge 三层角度是否连续。
- 小腿四连杆映射是否出现长期顶到限幅的情况。

注意：
如果当前脚本里还在使用 theta_hip / theta1 / motor2，
说明它还停留在旧 bridge 字段名，需要后续同步到：
  hip_motor / thigh_motor / calf_motor
"""
import sys
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from single_leg_slider_controller import SingleLegSliderController
from real_leg_adapter import RealLegCommandAdapter


MJCF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"


def fmt_layer(d):
    return "  ".join(f"{k}={v:8.2f}" for k, v in d.items())


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    controller = SingleLegSliderController(model)
    controller.initialize_targets(data)

    adapter = RealLegCommandAdapter()

    print("dry-run real leg commands")
    print("不会连接真实电机，只打印三层角度")
    print("motor config:", adapter.motor_config)
    print()

    dt = 0.02
    next_print_t = 0.0

    while data.time < controller.total_time:
        ctrl, info = controller.compute_control(data)
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)

        if data.time >= next_print_t:
            cmd = adapter.q_des_to_command(info["q_des"])

            print(f"t={data.time:5.2f}s  phase={info['phase']}")
            print("  sim_joint_deg:    ", fmt_layer(cmd["sim_joint_deg"]))
            print("  common_joint_deg: ", fmt_layer(cmd["common_joint_deg"]))
            print("  bridge_cmd_deg:   ", fmt_layer(cmd["bridge_cmd_deg"]))
            print()

            next_print_t += dt


if __name__ == "__main__":
    main()
