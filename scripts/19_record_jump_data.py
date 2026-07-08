import sys
from pathlib import Path

import mujoco
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model


MJCF_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_freebase_actuated.xml"
OUT_PNG = PROJECT_ROOT / "jump_record.png"
OUT_NPZ = PROJECT_ROOT / "jump_record_data.npz"

JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

TORQUE_LIMIT = 8.0

PREPARE_TIME = 1.0
PUSH_TIME = 0.5
FLIGHT_TIME = 0.5
LANDING_TIME = 3


# 落地阶段中，有多少比例用于“吸收冲击”。
# 0.45 表示 landing 时间的前 45% 用来从伸腿姿态过渡到软着陆姿态。
LANDING_ABSORB_RATIO = 0.45

# 两次重复播放之间的等待时间，单位秒。
REPLAY_PAUSE_TIME = 8.0

# 一次完整动作的总时间。
TOTAL_TIME = PREPARE_TIME + PUSH_TIME + FLIGHT_TIME + LANDING_TIME

KP_PUSH = np.array([16.0, 42.0, 42.0] * 4)
KD_PUSH = np.array([0.8, 1.6, 1.6] * 4)

KP_LAND = np.array([14.0, 24.0, 24.0] * 4)
KD_LAND = np.array([1.4, 3.2, 3.2] * 4)

KP_RECOVER = np.array([18.0, 30.0, 30.0] * 4)
KD_RECOVER = np.array([1.0, 2.4, 2.4] * 4)


def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return 3.0 * s * s - 2.0 * s * s * s


def make_pose(hip, thigh, calf):
    return np.array([hip, thigh, calf] * 4, dtype=float)


def get_joint_q_qd(model, data):
    q = np.zeros(12)
    qd = np.zeros(12)

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        q[i] = data.qpos[model.jnt_qposadr[joint_id]]
        qd[i] = data.qvel[model.jnt_dofadr[joint_id]]

    return q, qd


def get_actual_tau(model, data):
    tau = np.zeros(12)

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        dof_id = model.jnt_dofadr[joint_id]
        tau[i] = data.qfrc_actuator[dof_id]

    return tau


def set_initial_pose(model, data, q_init):
    mujoco.mj_resetData(model, data)

    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = q_init[i]

    mujoco.mj_forward(model, data)


def get_target_pose(t, q_stand, q_crouch, q_extend, q_soft_land):
    if t < PREPARE_TIME:
        a = smoothstep(t / PREPARE_TIME)
        return (1.0 - a) * q_stand + a * q_crouch, KP_LAND, KD_LAND, "prepare"

    if t < PREPARE_TIME + PUSH_TIME:
        s = (t - PREPARE_TIME) / PUSH_TIME
        a = smoothstep(s)
        return (1.0 - a) * q_crouch + a * q_extend, KP_PUSH, KD_PUSH, "push"

    if t < PREPARE_TIME + PUSH_TIME + FLIGHT_TIME:
        return q_extend, 0.45 * KP_LAND, KD_LAND, "flight"

    if t < TOTAL_TIME:
        landing_t = t - PREPARE_TIME - PUSH_TIME - FLIGHT_TIME
        absorb_time = LANDING_TIME * LANDING_ABSORB_RATIO

        if landing_t < absorb_time:
            s = landing_t / absorb_time
            a = smoothstep(s)
            return (1.0 - a) * q_extend + a * q_soft_land, KP_LAND, KD_LAND, "landing_absorb"

        s = (landing_t - absorb_time) / (LANDING_TIME - absorb_time)
        a = smoothstep(s)
        return (1.0 - a) * q_soft_land + a * q_stand, KP_RECOVER, KD_RECOVER, "landing_recover"

    return q_stand, KP_RECOVER, KD_RECOVER, "done"


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    robot_model = RealGo2Model()

    q_stand = robot_model.q0[7:19].copy()
    q_crouch = make_pose(0.0, 1.25, -2.35)
    q_extend = make_pose(0.0, 0.35, -0.95)
    q_soft_land = make_pose(0.0, 1.05, -2.05)

    set_initial_pose(model, data, q_stand)

    sim_time = TOTAL_TIME + 1.0
    steps = int(sim_time / model.opt.timestep)

    times = []
    base_zs = []
    base_vzs = []
    contacts = []
    max_ctrls = []
    max_actual_taus = []
    phase_ids = []

    phase_to_id = {
        "prepare": 0,
        "push": 1,
        "flight": 2,
        "landing_absorb": 3,
        "landing_recover": 4,
        "done": 5,
    }

    for _ in range(steps):
        t = data.time

        q_des, kp, kd, phase = get_target_pose(
            t,
            q_stand,
            q_crouch,
            q_extend,
            q_soft_land,
        )

        q, qd = get_joint_q_qd(model, data)

        ctrl = kp * (q_des - q) - kd * qd
        ctrl = np.clip(ctrl, -TORQUE_LIMIT, TORQUE_LIMIT)

        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)

        actual_tau = get_actual_tau(model, data)

        times.append(t)
        base_zs.append(float(data.qpos[2]))
        base_vzs.append(float(data.qvel[2]))
        contacts.append(int(data.ncon))
        max_ctrls.append(float(np.max(np.abs(ctrl))))
        max_actual_taus.append(float(np.max(np.abs(actual_tau))))
        phase_ids.append(phase_to_id[phase])

    times = np.array(times)
    base_zs = np.array(base_zs)
    base_vzs = np.array(base_vzs)
    contacts = np.array(contacts)
    max_ctrls = np.array(max_ctrls)
    max_actual_taus = np.array(max_actual_taus)
    phase_ids = np.array(phase_ids)

    print("simulation finished")
    print("max base z:", float(np.max(base_zs)))
    print("min base z:", float(np.min(base_zs)))
    print("final base z:", float(base_zs[-1]))
    print("max command ctrl:", float(np.max(max_ctrls)))
    print("max actual tau:", float(np.max(max_actual_taus)))
    print("zero contact steps:", int(np.sum(contacts == 0)))

    np.savez(
        OUT_NPZ,
        times=times,
        base_zs=base_zs,
        base_vzs=base_vzs,
        contacts=contacts,
        max_ctrls=max_ctrls,
        max_actual_taus=max_actual_taus,
        phase_ids=phase_ids,
    )

    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(times, base_zs)
    axes[0].axhline(0.25, linestyle="--", color="gray")
    axes[0].set_ylabel("base z [m]")
    axes[0].grid(True)

    axes[1].plot(times, base_vzs)
    axes[1].axhline(0.0, linestyle="--", color="gray")
    axes[1].set_ylabel("base vz [m/s]")
    axes[1].grid(True)

    axes[2].plot(times, contacts)
    axes[2].set_ylabel("contacts")
    axes[2].grid(True)

    axes[3].plot(times, max_ctrls, label="command ctrl")
    axes[3].plot(times, max_actual_taus, linestyle="--", label="actual tau")
    axes[3].axhline(TORQUE_LIMIT, linestyle=":", color="red")
    axes[3].set_ylabel("max torque [N*m]")
    axes[3].set_xlabel("time [s]")
    axes[3].legend()
    axes[3].grid(True)

    fig.suptitle("GO2 jump record")
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=150)

    print("saved plot:", OUT_PNG)
    print("saved data:", OUT_NPZ)


if __name__ == "__main__":
    main()
