import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model


URDF_PATH = PROJECT_ROOT / "models" / "custom_robot" / "GO2.urdf"
OUT_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_freebase_actuated.xml"
TMP_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_compiled_from_urdf.xml"

JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


def main():
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    fixed_model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    mujoco.mj_saveLastXML(str(TMP_PATH), fixed_model)

    robot_model = RealGo2Model()
    pin_mass = sum(inertia.mass for inertia in robot_model.model.inertias)
    mujoco_mass = float(np.sum(fixed_model.body_mass))
    missing_base_mass = max(0.1, pin_mass - mujoco_mass)

    tree = ET.parse(TMP_PATH)
    root = tree.getroot()

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")

    worldbody = root.find("worldbody")
    old_bodies = list(worldbody)

    for child in old_bodies:
        worldbody.remove(child)

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "pos": "0 0 0",
            "size": "3 3 0.05",
            "rgba": "0.8 0.8 0.8 1",
        },
    )

    floating_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "floating_base",
            "pos": "0 0 0.35",
        },
    )

    ET.SubElement(floating_body, "freejoint", {"name": "floating_base_joint"})

    ET.SubElement(
        floating_body,
        "inertial",
        {
            "pos": "0 0 0",
            "mass": f"{missing_base_mass:.6f}",
            "diaginertia": "0.03 0.10 0.11",
        },
    )

    for child in old_bodies:
        floating_body.append(child)

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    else:
        actuator.clear()

    for joint_name in JOINT_ORDER:
        motor_name = joint_name.replace("_joint", "_motor")
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": motor_name,
                "joint": joint_name,
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": "-8 8",
            },
        )

    tree.write(OUT_PATH, encoding="utf-8", xml_declaration=True)

    check_model = mujoco.MjModel.from_xml_path(str(OUT_PATH))

    print("generated:", OUT_PATH)
    print("nq:", check_model.nq)
    print("nv:", check_model.nv)
    print("nu:", check_model.nu)
    print("njnt:", check_model.njnt)
    print("total mass:", float(np.sum(check_model.body_mass)))


if __name__ == "__main__":
    main()
