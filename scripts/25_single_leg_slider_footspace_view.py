# ============================================================
# 25 脚本用途
# ============================================================
#
# 这个脚本是“单腿导轨仿真生成 + 调参 + viewer”入口。
#
# 它负责:
# 1. 从整机 GO2.urdf 裁出 RL 左后腿
# 2. 新增 rail_base、slider_link、slider_z_joint
# 3. 把 motor0/RL_hip_joint 放到 slider_link 上
# 4. 生成 MuJoCo MJCF
# 5. 给 MJCF 添加地面、导轨、滑块、腿部 visual mesh
# 6. 给 3 个关节添加 motor actuator
# 7. 规划 motor0 正下方的足端竖直轨迹
# 8. 用 IK 把足端目标转换成关节角
# 9. 用关节 PD 控制单腿跳跃
#
# 后面如果要改结构参数，比如:
# - slider 质量
# - motor0 位置
# - 足端目标偏移
# - mesh 显示
#
# 就改这个 25 脚本。
#
# 如果只想看控制怎么调用，去看:
# scripts/26_view_single_leg_control_only.py
# src/single_leg_slider_controller.py


import copy
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from single_leg_slider_controller import (
    FOOT_CONTACT_GEOM,
    SingleLegSliderController,
)


SRC_URDF = PROJECT_ROOT / "models" / "custom_robot" / "GO2.urdf"
OUT_URDF = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider.urdf"
OUT_MJCF = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"

LEG_LINKS = ["RL_hip", "RL_thigh", "RL_calf", "RL_foot"]
LEG_JOINTS = ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RL_foot_joint"]
MOTOR_JOINTS = ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]


# ============================================================
# 结构和动作参数
# ============================================================

# 滑块 + motor0 + 安装件的等效质量，单位 kg。
SLIDER_MASS = 0.2

# motor0/RL_hip_joint 在 slider_link 坐标系下的位置。
MOTOR0_XY_ON_SLIDER = np.array([0.0, 0.0])

# 足端目标相对 motor0 正下方的水平偏移。
FOOT_XY_OFFSET_FROM_MOTOR0 = np.array([0.0, 0.0790492])

# 动作时间。
PREPARE_TIME = 1.0
PUSH_TIME = 0.50
FLIGHT_TIME = 0.50
LANDING_TIME = 3.0
LANDING_ABSORB_RATIO = 0.70
REPLAY_PAUSE_TIME = 10.0

# 足端 z 方向轨迹。
CROUCH_DZ = 0.08
EXTEND_DZ = -0.10
SOFT_LAND_DZ = 0.04

TORQUE_LIMIT = 8.0


def indent(elem, level=0):
    space = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = space + "  "
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = space
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = space


def rewrite_mesh_paths(link):
    for mesh in link.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", "../custom_robot/meshes/" + Path(filename).name)


def make_simple_link(name, mass):
    link = ET.Element("link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": str(mass)})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "0.001",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.001",
            "iyz": "0",
            "izz": "0.001",
        },
    )
    return link


def make_slider_joint():
    joint = ET.Element("joint", {"name": "slider_z_joint", "type": "prismatic"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0.35", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": "rail_base"})
    ET.SubElement(joint, "child", {"link": "slider_link"})
    ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": "-0.30",
            "upper": "0.60",
            "effort": "100",
            "velocity": "5",
        },
    )
    return joint


def get_joint_origin_xyz(joint):
    origin = joint.find("origin")
    if origin is None:
        return np.zeros(3)
    return np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])


def set_joint_origin_xyz(joint, xyz):
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin", {"rpy": "0 0 0"})
    origin.set("xyz", f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}")


def generate_single_leg_urdf():
    """
    从整机 GO2.urdf 裁出 RL 左后腿，并加导轨滑块。
    原始 URDF 不会被修改。
    """

    src_root = ET.parse(SRC_URDF).getroot()
    out_root = ET.Element("robot", {"name": "RL_single_leg_slider"})

    for material in src_root.findall("material"):
        out_root.append(copy.deepcopy(material))

    out_root.append(make_simple_link("rail_base", 0.1))
    out_root.append(make_simple_link("slider_link", SLIDER_MASS))
    out_root.append(make_slider_joint())

    for link_name in LEG_LINKS:
        link = src_root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise RuntimeError(f"missing link: {link_name}")

        new_link = copy.deepcopy(link)
        rewrite_mesh_paths(new_link)
        out_root.append(new_link)

    for joint_name in LEG_JOINTS:
        joint = src_root.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise RuntimeError(f"missing joint: {joint_name}")

        new_joint = copy.deepcopy(joint)

        if joint_name == "RL_hip_joint":
            new_joint.find("parent").set("link", "slider_link")
            xyz = get_joint_origin_xyz(new_joint)
            xyz[0] = MOTOR0_XY_ON_SLIDER[0]
            xyz[1] = MOTOR0_XY_ON_SLIDER[1]
            set_joint_origin_xyz(new_joint, xyz)

        out_root.append(new_joint)

    OUT_URDF.parent.mkdir(parents=True, exist_ok=True)
    indent(out_root)
    ET.ElementTree(out_root).write(OUT_URDF, encoding="utf-8", xml_declaration=True)


def generate_mjcf():
    """
    从单腿 URDF 生成 MuJoCo MJCF，并补:
    - 地面
    - 导轨/滑块 visual
    - 腿部 visual mesh
    - 足端接触球命名
    - 3 个 motor actuator
    """

    model = mujoco.MjModel.from_xml_path(str(OUT_URDF))
    mujoco.mj_saveLastXML(str(OUT_MJCF), model)

    tree = ET.parse(OUT_MJCF)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", "../custom_robot/meshes")

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(1, asset)

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

    rail_fromto = (
        f"{MOTOR0_XY_ON_SLIDER[0]:.6f} {MOTOR0_XY_ON_SLIDER[1]:.6f} 0.0 "
        f"{MOTOR0_XY_ON_SLIDER[0]:.6f} {MOTOR0_XY_ON_SLIDER[1]:.6f} 0.9"
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "rail_visual",
            "type": "capsule",
            "fromto": rail_fromto,
            "size": "0.012",
            "rgba": "0.2 0.2 0.2 1",
            "contype": "0",
            "conaffinity": "0",
            "group": "2",
        },
    )

    visual_meshes = {
        "RL_hip": "RL_hip.STL",
        "RL_thigh": "RL_thigh.STL",
        "RL_calf": "RL_calf.STL",
        "RL_foot": "RL_foot.STL",
    }

    for body_name, filename in visual_meshes.items():
        mesh_name = body_name + "_visual_mesh"
        ET.SubElement(asset, "mesh", {"name": mesh_name, "file": filename})

        body = root.find(f".//body[@name='{body_name}']")
        if body is not None:
            ET.SubElement(
                body,
                "geom",
                {
                    "name": body_name + "_visual",
                    "type": "mesh",
                    "mesh": mesh_name,
                    "contype": "0",
                    "conaffinity": "0",
                    "rgba": "0.75 0.75 0.78 1",
                    "group": "2",
                },
            )

    calf_body = root.find(".//body[@name='RL_calf']")
    if calf_body is not None:
        for geom in calf_body.findall("geom"):
            if geom.get("name") is None and geom.get("size") is not None:
                geom.set("name", FOOT_CONTACT_GEOM)
                break

    slider_body = root.find(".//body[@name='slider_link']")
    if slider_body is not None:
        ET.SubElement(
            slider_body,
            "geom",
            {
                "name": "slider_visual",
                "type": "box",
                "size": "0.04 0.04 0.03",
                "rgba": "0.1 0.3 0.6 1",
                "contype": "0",
                "conaffinity": "0",
                "group": "2",
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
                "ctrlrange": f"{-TORQUE_LIMIT} {TORQUE_LIMIT}",
            },
        )

    tree.write(OUT_MJCF, encoding="utf-8", xml_declaration=True)


def apply_controller_params(controller):
    """
    把 25 脚本顶部的调参值同步给控制器。
    """

    controller.target_foot_xy = FOOT_XY_OFFSET_FROM_MOTOR0.copy()

    controller.prepare_time = PREPARE_TIME
    controller.push_time = PUSH_TIME
    controller.flight_time = FLIGHT_TIME
    controller.landing_time = LANDING_TIME
    controller.landing_absorb_ratio = LANDING_ABSORB_RATIO
    controller.update_total_time()

    controller.crouch_dz = CROUCH_DZ
    controller.extend_dz = EXTEND_DZ
    controller.soft_land_dz = SOFT_LAND_DZ


def main():
    print("regenerate single-leg URDF/MJCF...")
    generate_single_leg_urdf()
    generate_mjcf()

    model = mujoco.MjModel.from_xml_path(str(OUT_MJCF))
    data = mujoco.MjData(model)

    controller = SingleLegSliderController(model)
    apply_controller_params(controller)
    controller.initialize_targets(data)

    print("model loaded")
    print("nq:", model.nq, "nv:", model.nv, "nu:", model.nu, "ngeom:", model.ngeom)
    print("slider mass:", SLIDER_MASS)
    print("motor0 xy on slider:", MOTOR0_XY_ON_SLIDER)
    print("foot xy offset from motor0:", FOOT_XY_OFFSET_FROM_MOTOR0)
    print("p_stand:", controller.p_stand)
    print("q_stand:", controller.q_stand)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.25]

        replay_start = time.time()

        while viewer.is_running():
            elapsed = time.time() - replay_start

            if elapsed > controller.total_time + REPLAY_PAUSE_TIME:
                controller.reset_for_replay(data)
                replay_start = time.time()
                print("replay")
                viewer.sync()
                time.sleep(0.3)
                continue

            ctrl, info = controller.compute_control(data)

            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()