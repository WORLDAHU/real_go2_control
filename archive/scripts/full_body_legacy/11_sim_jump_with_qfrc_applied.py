import sys
from pathlib import Path

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from jump_controller import JumpController


URDF_PATH = PROJECT_ROOT / "models" / "custom_robot" / "GO2.urdf"

TORQUE_JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


def apply_joint_torques(model, data, tau):
    data.qfrc_applied[:] = 0.0

    for tau_i, joint_name in enumerate(TORQUE_JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

        if joint_id < 0:
            raise RuntimeError(f"MuJoCo 里找不到关节: {joint_name}")

        dof_id = model.jnt_dofadr[joint_id]
        data.qfrc_applied[dof_id] = tau[tau_i]


def main():
    mj_model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    mj_data = mujoco.MjData(mj_model)

    robot_model = RealGo2Model()
    jump_controller = JumpController(robot_model)

    print("MuJoCo model loaded")
    print("nq:", mj_model.nq)
    print("nv:", mj_model.nv)
    print("nu:", mj_model.nu)
    print("njnt:", mj_model.njnt)

    if mj_model.nv == 12:
        print()
        print("注意: 当前 nv=12，说明这是固定基座模型。")
        print("这一步先验证 12 个关节能接收力矩，暂时不期待机身真的跳起来。")
        print()

    dt = mj_model.opt.timestep
    sim_time = jump_controller.total_duration + 0.5
    steps = int(sim_time / dt)

    max_tau_seen = 0.0

    for step in range(steps):
        t = step * dt

        phase, force_per_leg, tau = jump_controller.compute_smooth_torque_at_time(t)
        tau = np.asarray(tau, dtype=float)

        tau = np.clip(tau, -80.0, 80.0)

        apply_joint_torques(mj_model, mj_data, tau)
        mujoco.mj_step(mj_model, mj_data)

        max_tau_seen = max(max_tau_seen, float(np.max(np.abs(tau))))

        if step % 200 == 0:
            print(
                f"t={t:5.3f}  phase={phase:8s}  "
                f"force/leg={force_per_leg:8.3f}  "
                f"max|tau|={np.max(np.abs(tau)):7.3f}  "
                f"qpos[0:3]={mj_data.qpos[:3]}"
            )

    print()
    print("simulation finished")
    print("max torque used:", max_tau_seen)


if __name__ == "__main__":
    main()