from pathlib import Path
import copy
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]

SRC_URDF = PROJECT_ROOT / "models" / "custom_robot" / "GO2.urdf"
OUT_URDF = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider.urdf"

KEEP_LINKS = [
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RL_foot",
]

KEEP_JOINTS = [
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RL_foot_joint",
]


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
    """
    输出 URDF 在 models/single_leg/ 下面。

    原来的 mesh 文件在:
        models/custom_robot/meshes/

    所以这里把 mesh 路径统一改成:
        ../custom_robot/meshes/xxx.STL
    """

    for mesh in link.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None:
            continue

        mesh_name = Path(filename).name
        mesh.set("filename", f"../custom_robot/meshes/{mesh_name}")


def make_simple_link(name, mass="0.1"):
    link = ET.Element("link", {"name": name})

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": mass})
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
            "lower": "-0.25",
            "upper": "0.45",
            "effort": "100",
            "velocity": "5",
        },
    )

    return joint


def main():
    if not SRC_URDF.exists():
        raise FileNotFoundError(f"找不到原始 URDF: {SRC_URDF}")

    src_root = ET.parse(SRC_URDF).getroot()

    out_root = ET.Element("robot", {"name": "RL_single_leg_slider"})

    # 复制 material，避免 visual 颜色引用丢失。
    for material in src_root.findall("material"):
        out_root.append(copy.deepcopy(material))

    # 新增导轨固定端和滑块。
    out_root.append(make_simple_link("rail_base", mass="0.1"))
    out_root.append(make_simple_link("slider_link", mass="0.1"))
    out_root.append(make_slider_joint())

    # 复制左后腿 links。
    for link_name in KEEP_LINKS:
        link = src_root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise RuntimeError(f"原 URDF 中找不到 link: {link_name}")

        new_link = copy.deepcopy(link)
        rewrite_mesh_paths(new_link)
        out_root.append(new_link)

    # 复制左后腿 joints。
    for joint_name in KEEP_JOINTS:
        joint = src_root.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise RuntimeError(f"原 URDF 中找不到 joint: {joint_name}")

        new_joint = copy.deepcopy(joint)

        # 原来是 base_link -> RL_hip。
        # 单腿台架里改成 slider_link -> RL_hip。
        if joint_name == "RL_hip_joint":
            parent = new_joint.find("parent")
            parent.set("link", "slider_link")

        out_root.append(new_joint)

    indent(out_root)

    OUT_URDF.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(out_root)
    tree.write(OUT_URDF, encoding="utf-8", xml_declaration=True)

    print("generated:", OUT_URDF)
    print()
    print("下一步可以检查 MuJoCo 是否能加载:")
    print("python scripts/20_check_single_leg_urdf.py")


if __name__ == "__main__":
    main()
