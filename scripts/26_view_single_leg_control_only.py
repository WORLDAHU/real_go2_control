import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

# ============================================================
# 路径设置
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from single_leg_slider_controller import SingleLegSliderController


# ============================================================
# 模型路径
# ============================================================

# 这个 MJCF 由 25 脚本生成。
# 里面已经包含:
# - 单腿结构
# - slider_z_joint 导轨自由度
# - 3 个 motor actuator
# - 地面
# - 可视化 mesh
MJCF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"

# 每次动作结束后，等待多久再重新播放。
REPLAY_PAUSE_TIME = 4.0


def main():
    # 读取 MuJoCo 模型。
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    # 创建单腿导轨控制器。
    # 控制器内部负责:
    # 1. 规划足端竖直轨迹
    # 2. IK 求关节角
    # 3. PD 输出 3 个电机力矩
    controller = SingleLegSliderController(model)

    # 初始化控制器目标点，并把机器人放到初始姿态。
    controller.initialize_targets(data)

    print("control-only viewer")
    print("total time:", controller.total_time)
    print("close window to stop")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 初始相机视角。
        # 这里没有写跟随逻辑，所以可以用鼠标自由拖动视角。
        viewer.cam.distance = 1.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.25]

        replay_start = time.time()

        while viewer.is_running():
            elapsed = time.time() - replay_start

            # 一次动作结束后，等待几秒，然后重置并重播。
            if elapsed > controller.total_time + REPLAY_PAUSE_TIME:
                controller.reset_for_replay(data)
                replay_start = time.time()
                print("replay")
                viewer.sync()
                time.sleep(0.3)
                continue

            # 这是最核心的一行:
            # 根据当前 MuJoCo 状态 data，计算 3 个电机力矩 ctrl。
            ctrl, info = controller.compute_control(data)

            # 把控制器输出写入 MuJoCo actuator。
            data.ctrl[:] = ctrl

            # 推进一步仿真。
            mujoco.mj_step(model, data)

            # 刷新 viewer 画面。
            viewer.sync()

            # 让播放速度接近真实时间。
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()