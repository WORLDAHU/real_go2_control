import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from jump_controller import JumpController


MJCF_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_freebase_actuated.xml"

JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

TORQUE_SIGN = -1.0
TORQUE_LIMIT = 8.0

KP = np.array([18.0, 28.0, 28.0] * 4)
KD = np.array([0.8, 1.2, 1.2] * 4)

PUSH_ACC = 50.0


def get_joint_q_qd(model, data):
    q = np.zeros(12)
    qd = np.zeros(12)

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        q[i] = data.qpos[model.jnt_qposadr[joint_id]]
        qd[i] = data.qvel[model.jnt_dofadr[joint_id]]

    return q, qd


def set_initial_pose(model, data, q_des):
    mujoco.mj_resetData(model, data)

    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = q_des[i]

    mujoco.mj_forward(model, data)


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    robot_model = RealGo2Model()
    jump_controller = JumpController(robot_model)

    jump_controller.push_acc = PUSH_ACC
    jump_controller.push_duration = 0.22
    jump_controller.flight_duration = 0.35
    jump_controller.total_duration = (
        jump_controller.prepare_duration
        + jump_controller.push_duration
        + jump_controller.flight_duration
        + jump_controller.landing_duration
    )

    q_des = robot_model.q0[7:19].copy()
    set_initial_pose(model, data, q_des)

    sim_time = jump_controller.total_duration + 1.0
    #start_wall_time = time.time()

    print("打开 MuJoCo viewer")
    print("push_acc:", PUSH_ACC)
    print("torque limit:", TORQUE_LIMIT)
    print("关闭窗口即可结束")
    print()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_wall_time = time.time()
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.25]

        while viewer.is_running():
            elapsed = time.time() - start_wall_time

            if elapsed > sim_time:
                set_initial_pose(model, data, q_des)
                start_wall_time = time.time()
                print("replay jump")
                viewer.sync()
                time.sleep(0.3)
                continue

            t = data.time

            phase, force_per_leg, tau_jump = jump_controller.compute_smooth_torque_at_time(t)
            tau_jump = np.asarray(tau_jump, dtype=float)

            q, qd = get_joint_q_qd(model, data)
            tau_pd = KP * (q_des - q) - KD * qd

            ctrl = TORQUE_SIGN * tau_jump + tau_pd
            ctrl = np.clip(ctrl, -TORQUE_LIMIT, TORQUE_LIMIT)

            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

            # 自由视角：不再每帧强制跟随机器人
            viewer.sync()

            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
