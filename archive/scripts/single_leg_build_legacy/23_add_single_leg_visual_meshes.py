from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MJCF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider_mujoco.xml"

MESH_DIR = "../custom_robot/meshes"

VISUAL_MESHES = {
    "RL_hip": "RL_hip.STL",
    "RL_thigh": "RL_thigh.STL",
    "RL_calf": "RL_calf.STL",
    "RL_foot": "RL_foot.STL",
}


def find_body(root, body_name):
    for body in root.findall(".//body"):
        if body.get("name") == body_name:
            return body
    return None


def main():
    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)

    compiler.set("meshdir", MESH_DIR)

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(1, asset)

    existing_mesh_names = {
        mesh.get("name")
        for mesh in asset.findall("mesh")
        if mesh.get("name")
    }

    for body_name, filename in VISUAL_MESHES.items():
        mesh_name = body_name + "_visual_mesh"

        if mesh_name not in existing_mesh_names:
            ET.SubElement(
                asset,
                "mesh",
                {
                    "name": mesh_name,
                    "file": filename,
                },
            )

        body = find_body(root, body_name)
        if body is None:
            print("skip, body not found:", body_name)
            continue

        visual_name = body_name + "_visual"
        already_added = any(
            geom.get("name") == visual_name
            for geom in body.findall("geom")
        )

        if already_added:
            continue

        ET.SubElement(
            body,
            "geom",
            {
                "name": visual_name,
                "type": "mesh",
                "mesh": mesh_name,
                "contype": "0",
                "conaffinity": "0",
                "rgba": "0.75 0.75 0.78 1",
                "group": "2",
            },
        )

    worldbody = root.find("worldbody")

    if not any(g.get("name") == "rail_visual" for g in worldbody.findall("geom")):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "rail_visual",
                "type": "capsule",
                "fromto": "0.08 0 0.0 0.08 0 0.9",
                "size": "0.012",
                "rgba": "0.2 0.2 0.2 1",
                "contype": "0",
                "conaffinity": "0",
                "group": "2",
            },
        )

    slider_body = find_body(root, "slider_link")
    if slider_body is not None:
        if not any(g.get("name") == "slider_visual" for g in slider_body.findall("geom")):
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

    tree.write(MJCF_PATH, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

    print("updated:", MJCF_PATH)
    print("nmesh:", model.nmesh)
    print("ngeom:", model.ngeom)
    print("nu:", model.nu)


if __name__ == "__main__":
    main()
