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
OUT_PNG = PROJECT_ROOT / "jump_base_height.png"

JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

TORQUE_SIGN = -1.0


def set_initial_pose(model, data, robot_model):
    mujoco.mj_resetData(model, data)

    # freejoint 的 qpos 顺序:
    # x, y, z, qw, qx, qy, qz
    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_id = model.jnt_qposadr[joint_id]
        data.qpos[qpos_id] = robot_model.q0[7 + i]

    mujoco.mj_forward(model, data)


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    robot_model = RealGo2Model()
    jump_controller = JumpController(robot_model)

    set_initial_pose(model, data, robot_model)

    print("MuJoCo model loaded")
    print("nq:", model.nq)
    print("nv:", model.nv)
    print("nu:", model.nu)
    print("start base z:", data.qpos[2])
    print("start contacts:", data.ncon)
    print()

    dt = model.opt.timestep
    sim_time = jump_controller.total_duration + 0.6
    steps = int(sim_time / dt)

    times = []
    base_zs = []
    base_vzs = []
    max_ctrls = []
    phases = []

    for step in range(steps):
        t = step * dt

        phase, force_per_leg, tau = jump_controller.compute_smooth_torque_at_time(t)
        tau = np.asarray(tau, dtype=float)

        ctrl = TORQUE_SIGN * tau
        ctrl = np.clip(ctrl, -8.0, 8.0)

        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)

        times.append(t)
        base_zs.append(float(data.qpos[2]))
        base_vzs.append(float(data.qvel[2]))
        max_ctrls.append(float(np.max(np.abs(ctrl))))
        phases.append(phase)

        if step % 200 == 0:
            print(
                f"t={t:5.3f}  phase={phase:8s}  "
                f"force/leg={force_per_leg:8.3f}  "
                f"z={data.qpos[2]:7.4f}  "
                f"vz={data.qvel[2]:8.4f}  "
                f"contacts={data.ncon}  "
                f"max|ctrl|={np.max(np.abs(ctrl)):7.3f}"
            )

    times = np.array(times)
    base_zs = np.array(base_zs)
    base_vzs = np.array(base_vzs)
    max_ctrls = np.array(max_ctrls)

    print()
    print("simulation finished")
    print("min base z:", float(np.min(base_zs)))
    print("max base z:", float(np.max(base_zs)))
    print("final base z:", float(base_zs[-1]))
    print("max control torque:", float(np.max(max_ctrls)))

    plt.figure(figsize=(10, 5))
    plt.plot(times, base_zs, label="base z")
    plt.xlabel("time [s]")
    plt.ylabel("base height z [m]")
    plt.title("Free-base GO2 jump simulation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150)

    print("saved plot:", OUT_PNG)


if __name__ == "__main__":
    main()
