"""
三层角度检查脚本。

用途：
- 不连接真实电机。
- 不打开 viewer。
- 在几个关键时刻打印：
    sim_joint_deg
    common_joint_deg
    bridge_cmd_deg

它主要用来检查角度命名和转换是否符合预期。

重点观察：
- hip_joint / hip_abduction 是否接近 0，确认腿平面没有被 IK 强行歪掉。
- knee_pitch 是否随蹲下/伸腿按预期增减。
- calf_motor_unclamped 是否明显超出 calf_motor 限幅。
"""
import sys
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from single_leg_slider_controller import SingleLegSliderController
from real_leg_adapter import RealLegCommandAdapter


MJCF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"


def fmt_sim(d):
    return (
        f"hip_joint={d['hip_joint']:8.2f}  "
        f"thigh_joint={d['thigh_joint']:8.2f}  "
        f"calf_joint={d['calf_joint']:8.2f}"
    )


def fmt_common(d):
    return (
        f"hip_abduction={d['hip_abduction']:8.2f}  "
        f"thigh_pitch={d['thigh_pitch']:8.2f}  "
        f"knee_pitch={d['knee_pitch']:8.2f}"
    )


def fmt_bridge(d):
    return (
        f"hip_motor={d['hip_motor']:8.2f}  "
        f"thigh_motor={d['thigh_motor']:8.2f}  "
        f"calf_motor={d['calf_motor']:8.2f}  "
        f"calf_motor_unclamped={d['calf_motor_unclamped']:8.2f}"
    )


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    controller = SingleLegSliderController(model)
    controller.initialize_targets(data)

    adapter = RealLegCommandAdapter()

    print("joint angle naming check")
    print("knee_pitch 正方向: 从 +Y 的位置看向 XZ 平面，逆时针为正")
    print()
    print("sim_joint_deg -> common_joint_deg:")
    print("  hip_abduction =  1 * (hip_joint   - 0 deg)")
    print("  thigh_pitch   =  1 * (thigh_joint - 0 deg)")
    print("  knee_pitch    = -1 * (calf_joint  - 0 deg)")
    print()


    # 抽样时刻：
    # 0.00：动作刚开始，检查初始站立姿态。
    # prepare_time：蹲下阶段结束。
    # prepare_time + push_time：蹬伸阶段结束。
    # prepare_time + push_time + flight_time：飞行阶段结束。
    # total_time：整段动作结束。
    sample_times = [
        0.00,
        controller.prepare_time,
        controller.prepare_time + controller.push_time,
        controller.prepare_time + controller.push_time + controller.flight_time,
        controller.total_time,
    ]

    next_idx = 0

    while data.time <= controller.total_time + 1e-6:
        ctrl, info = controller.compute_control(data)
        data.ctrl[:] = ctrl

        if next_idx < len(sample_times) and data.time >= sample_times[next_idx] - 1e-6:
            cmd = adapter.q_des_to_command(info["q_des"])

            print(f"t={data.time:5.2f}s  phase={info['phase']}")
            print("  sim_joint_deg:    ", fmt_sim(cmd["sim_joint_deg"]))
            print("  common_joint_deg: ", fmt_common(cmd["common_joint_deg"]))
            print("  bridge_cmd_deg:   ", fmt_bridge(cmd["bridge_cmd_deg"]))
            print()

            next_idx += 1

        mujoco.mj_step(model, data)


if __name__ == "__main__":
    main()
