import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MJCF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"

MOTOR_JOINTS = [
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]

TORQUE_LIMIT = 8.0

PREPARE_TIME = 1.0
PUSH_TIME = 0.45
LANDING_TIME = 1.5
TOTAL_TIME = PREPARE_TIME + PUSH_TIME + LANDING_TIME
REPLAY_PAUSE_TIME = 13.0

KP_PREPARE = np.array([8.0, 18.0, 18.0])
KD_PREPARE = np.array([0.8, 2.0, 2.0])

KP_PUSH = np.array([10.0, 34.0, 34.0])
KD_PUSH = np.array([0.8, 2.2, 2.2])

KP_LAND = np.array([8.0, 16.0, 16.0])
KD_LAND = np.array([1.2, 3.5, 3.5])


def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return 3.0 * s * s - 2.0 * s * s * s


def get_joint_q_qd(model, data):
    q = np.zeros(3)
    qd = np.zeros(3)

    for i, joint_name in enumerate(MOTOR_JOINTS):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        q[i] = data.qpos[model.jnt_qposadr[joint_id]]
        qd[i] = data.qvel[model.jnt_dofadr[joint_id]]

    return q, qd


def set_initial_pose(model, data, q_stand):
    mujoco.mj_resetData(model, data)

    slider_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "slider_z_joint")
    data.qpos[model.jnt_qposadr[slider_id]] = 0.0

    for i, joint_name in enumerate(MOTOR_JOINTS):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = q_stand[i]

    mujoco.mj_forward(model, data)


def get_target(t, q_stand, q_crouch, q_extend, q_soft_land):
    if t < PREPARE_TIME:
        a = smoothstep(t / PREPARE_TIME)
        return (1.0 - a) * q_stand + a * q_crouch, KP_PREPARE, KD_PREPARE, "prepare", 3.0

    if t < PREPARE_TIME + PUSH_TIME:
        s = (t - PREPARE_TIME) / PUSH_TIME
        a = smoothstep(s)
        q_des = (1.0 - a) * q_crouch + a * q_extend

        # push 末尾降低一点刚度，避免切换瞬间过硬。
        taper = 1.0 - 0.30 * smoothstep((s - 0.75) / 0.25)
        return q_des, taper * KP_PUSH, KD_PUSH, "push", 8.0

    if t < TOTAL_TIME:
        s = (t - PREPARE_TIME - PUSH_TIME) / LANDING_TIME
        a = smoothstep(s)

        if s < 0.55:
            b = smoothstep(s / 0.55)
            return (1.0 - b) * q_extend + b * q_soft_land, KP_LAND, KD_LAND, "landing_absorb", 4.0

        b = smoothstep((s - 0.55) / 0.45)
        return (1.0 - b) * q_soft_land + b * q_stand, KP_PREPARE, KD_PREPARE, "landing_recover", 3.0

    return q_stand, KP_PREPARE, KD_PREPARE, "done", 2.0


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    q_stand = np.array([0.0, 0.9, -1.8])
    q_crouch = np.array([0.0, 1.25, -2.35])
    q_extend = np.array([0.0, 0.50, -1.20])
    q_soft_land = np.array([0.0, 1.15, -2.15])

    set_initial_pose(model, data, q_stand)

    print("single leg slider simulation")
    print("nu:", model.nu)
    print("q_stand:", q_stand)
    print("q_crouch:", q_crouch)
    print("q_extend:", q_extend)
    print("close window to stop")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.25]

        replay_start = time.time()

        while viewer.is_running():
            elapsed = time.time() - replay_start

            if elapsed > TOTAL_TIME + REPLAY_PAUSE_TIME:
                set_initial_pose(model, data, q_stand)
                replay_start = time.time()
                print("replay")
                viewer.sync()
                time.sleep(0.3)
                continue

            t = data.time
            q_des, kp, kd, phase, limit = get_target(
                t,
                q_stand,
                q_crouch,
                q_extend,
                q_soft_land,
            )

            q, qd = get_joint_q_qd(model, data)

            ctrl = kp * (q_des - q) - kd * qd
            ctrl = np.clip(ctrl, -limit, limit)

            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
