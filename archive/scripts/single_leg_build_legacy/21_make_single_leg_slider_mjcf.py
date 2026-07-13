from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


PROJECT_ROOT = Path(__file__).resolve().parents[1]

URDF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider.urdf"
OUT_XML = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"

MOTOR_JOINTS = [
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


def main():
    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    mujoco.mj_saveLastXML(str(OUT_XML), model)

    tree = ET.parse(OUT_XML)
    root = tree.getroot()

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")

    worldbody = root.find("worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "pos": "0 0 0",
            "size": "2 2 0.05",
            "rgba": "0.8 0.8 0.8 1",
        },
    )

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    else:
        actuator.clear()

    for joint_name in MOTOR_JOINTS:
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": joint_name.replace("_joint", "_motor"),
                "joint": joint_name,
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": "-8 8",
            },
        )

    tree.write(OUT_XML, encoding="utf-8", xml_declaration=True)

    check_model = mujoco.MjModel.from_xml_path(str(OUT_XML))

    print("generated:", OUT_XML)
    print("nq:", check_model.nq)
    print("nv:", check_model.nv)
    print("nu:", check_model.nu)
    print("njnt:", check_model.njnt)
    print("ngeom:", check_model.ngeom)


if __name__ == "__main__":
    main()
