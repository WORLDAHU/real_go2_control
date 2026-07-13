import sys
from pathlib import Path

import mujoco
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from jump_controller import JumpController


MJCF_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_freebase_actuated.xml"
OUT_PNG = PROJECT_ROOT / "jump_push_acc_sweep.png"

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

PUSH_ACC_LIST = [8.0, 20.0, 35.0, 50.0, 70.0]


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


def run_one(push_acc):
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    robot_model = RealGo2Model()
    jump_controller = JumpController(robot_model)

    jump_controller.push_acc = push_acc
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

    dt = model.opt.timestep
    sim_time = jump_controller.total_duration + 1.0
    steps = int(sim_time / dt)

    times = []
    base_zs = []
    contacts = []
    max_command_ctrl = 0.0
    max_actual_tau = 0.0

    for step in range(steps):
        t = step * dt

        phase, force_per_leg, tau_jump = jump_controller.compute_smooth_torque_at_time(t)
        tau_jump = np.asarray(tau_jump, dtype=float)

        q, qd = get_joint_q_qd(model, data)
        tau_pd = KP * (q_des - q) - KD * qd

        ctrl = TORQUE_SIGN * tau_jump + tau_pd
        ctrl = np.clip(ctrl, -TORQUE_LIMIT, TORQUE_LIMIT)

        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)


        actual_tau = []
        for joint_name in JOINT_ORDER:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            dof_id = model.jnt_dofadr[joint_id]
            actual_tau.append(data.qfrc_actuator[dof_id])

        actual_tau = np.array(actual_tau)

        times.append(t)
        base_zs.append(float(data.qpos[2]))
        contacts.append(int(data.ncon))
        max_command_ctrl = max(max_command_ctrl, float(np.max(np.abs(ctrl))))
        max_actual_tau = max(max_actual_tau, float(np.max(np.abs(actual_tau))))

    times = np.array(times)
    base_zs = np.array(base_zs)
    contacts = np.array(contacts)

    return {
    "push_acc": push_acc,
    "times": times,
    "base_zs": base_zs,
    "min_z": float(np.min(base_zs)),
    "max_z": float(np.max(base_zs)),
    "final_z": float(base_zs[-1]),
    "max_command_ctrl": max_command_ctrl,
    "max_actual_tau": max_actual_tau,
    "zero_contact_steps": int(np.sum(contacts == 0)),
    }


def main():
    results = []

    plt.figure(figsize=(10, 5))

    for push_acc in PUSH_ACC_LIST:
        result = run_one(push_acc)
        results.append(result)

        plt.plot(
            result["times"],
            result["base_zs"],
            label=f"push_acc={push_acc:g}",
        )

        print(
            f"push_acc={push_acc:5.1f}  "
            f"max_z={result['max_z']:.4f}  "
            f"final_z={result['final_z']:.4f}  "
            f"min_z={result['min_z']:.4f}  "
            f"max_command_ctrl={result['max_command_ctrl']:.2f}  "
            f"max_actual_tau={result['max_actual_tau']:.2f}  "
            f"zero_contact_steps={result['zero_contact_steps']}"
        )

    plt.axhline(0.25, linestyle="--", color="gray", label="start height")
    plt.xlabel("time [s]")
    plt.ylabel("base height z [m]")
    plt.title("Jump push_acc sweep")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150)

    print()
    print("saved plot:", OUT_PNG)


if __name__ == "__main__":
    main()
