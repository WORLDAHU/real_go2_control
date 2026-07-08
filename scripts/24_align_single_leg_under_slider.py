from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider.urdf"

# 1.0 表示完全把站立姿态下的足端对到 motor0 正下方。
# 0.5 表示只挪一半，更保守。
ALIGN_GAIN = 0.5

#改完后重新生成：
#python scripts/20_make_rl_single_leg_slider_urdf.py
#python scripts/21_make_single_leg_slider_mjcf.py
#python scripts/23_add_single_leg_visual_meshes.py



Q_STAND = np.array([0.0, 0.9, -1.8])

MOTOR_JOINTS = [
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


def get_origin_xyz(joint):
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

    xyz = origin.get("xyz", "0 0 0")
    return origin, np.array([float(v) for v in xyz.split()])


def main():
    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)

    for i, joint_name in enumerate(MOTOR_JOINTS):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = Q_STAND[i]

    mujoco.mj_forward(model, data)

    foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "RL_foot")
    foot_pos = data.xpos[foot_body_id].copy()

    print("current foot position:", foot_pos)
    print("current foot xy offset from motor0/slider:", foot_pos[:2])

    tree = ET.parse(URDF_PATH)
    root = tree.getroot()

    hip_joint = root.find("./joint[@name='RL_hip_joint']")
    if hip_joint is None:
        raise RuntimeError("找不到 RL_hip_joint")

    origin, xyz = get_origin_xyz(hip_joint)

    # 目标: 让站立姿态下 foot x/y 更靠近 0/0。
    # foot_xy 偏多少，就把整条腿反方向挪多少。
    delta_xy = -ALIGN_GAIN * foot_pos[:2]
    new_xyz = xyz.copy()
    new_xyz[0] += delta_xy[0]
    new_xyz[1] += delta_xy[1]

    origin.set("xyz", f"{new_xyz[0]:.6f} {new_xyz[1]:.6f} {new_xyz[2]:.6f}")

    tree.write(URDF_PATH, encoding="utf-8", xml_declaration=True)

    print()
    print("old RL_hip_joint origin xyz:", xyz)
    print("delta xy:", delta_xy)
    print("new RL_hip_joint origin xyz:", new_xyz)
    print()
    print("接下来重新生成 MJCF:")
    print("python scripts/21_make_single_leg_slider_mjcf.py")
    print("python scripts/23_add_single_leg_visual_meshes.py")


if __name__ == "__main__":
    main()
