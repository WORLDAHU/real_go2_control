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
OUT_PNG = PROJECT_ROOT / "jump_base_height_with_pd.png"

JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

TORQUE_SIGN = -1.0

KP = np.array([18.0, 28.0, 28.0] * 4)
KD = np.array([0.8, 1.2, 1.2] * 4)
TORQUE_LIMIT = 8.0


def get_joint_q_qd(model, data):
    q = np.zeros(12)
    qd = np.zeros(12)

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_id = model.jnt_qposadr[joint_id]
        dof_id = model.jnt_dofadr[joint_id]
        q[i] = data.qpos[qpos_id]
        qd[i] = data.qvel[dof_id]

    return q, qd


def set_initial_pose(model, data, q_des):
    mujoco.mj_resetData(model, data)

    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_id = model.jnt_qposadr[joint_id]
        data.qpos[qpos_id] = q_des[i]

    mujoco.mj_forward(model, data)


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    robot_model = RealGo2Model()
    jump_controller = JumpController(robot_model)

    q_des = robot_model.q0[7:19].copy()
    set_initial_pose(model, data, q_des)

    print("MuJoCo model loaded")
    print("nq:", model.nq)
    print("nv:", model.nv)
    print("nu:", model.nu)
    print("start base z:", data.qpos[2])
    print("start contacts:", data.ncon)
    print()

    dt = model.opt.timestep
    sim_time = jump_controller.total_duration + 1.2
    steps = int(sim_time / dt)

    times = []
    base_zs = []
    max_ctrls = []

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

        times.append(t)
        base_zs.append(float(data.qpos[2]))
        max_ctrls.append(float(np.max(np.abs(ctrl))))

        if step % 200 == 0:
            print(
                f"t={t:5.3f}  phase={phase:8s}  "
                f"z={data.qpos[2]:7.4f}  "
                f"vz={data.qvel[2]:8.4f}  "
                f"contacts={data.ncon}  "
                f"max|ctrl|={np.max(np.abs(ctrl)):7.3f}"
            )

    times = np.array(times)
    base_zs = np.array(base_zs)

    print()
    print("simulation finished")
    print("min base z:", float(np.min(base_zs)))
    print("max base z:", float(np.max(base_zs)))
    print("final base z:", float(base_zs[-1]))
    print("max control torque:", float(np.max(max_ctrls)))

    plt.figure(figsize=(10, 5))
    plt.plot(times, base_zs, label="base z with joint PD")
    plt.axhline(0.25, linestyle="--", color="gray", label="start height")
    plt.xlabel("time [s]")
    plt.ylabel("base height z [m]")
    plt.title("Free-base GO2 jump simulation with joint PD")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150)

    print("saved plot:", OUT_PNG)


if __name__ == "__main__":
    main()
