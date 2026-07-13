from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MJCF_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_freebase_actuated.xml"

MESH_DIR = "../custom_robot/meshes"

VISUAL_MESHES = {
    "FL_hip": "FL_hip.STL",
    "FL_thigh": "FL_thigh.STL",
    "FL_calf": "FL_calf.STL",

    "FR_hip": "FR_hip.STL",
    "FR_thigh": "FR_thigh.STL",
    "FR_calf": "FR_calf.STL",

    "RL_hip": "RL_hip.STL",
    "RL_thigh": "RL_thigh.STL",
    "RL_calf": "RL_calf.STL",

    "RR_hip": "RR_hip.STL",
    "RR_thigh": "RR_thigh.STL",
    "RR_calf": "RR_calf.STL",
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

        already_added = False
        for geom in body.findall("geom"):
            if geom.get("name") == body_name + "_visual":
                already_added = True
                break

        if already_added:
            continue

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

    tree.write(MJCF_PATH, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    print("updated:", MJCF_PATH)
    print("nq:", model.nq)
    print("nv:", model.nv)
    print("nu:", model.nu)
    print("nmesh:", model.nmesh)
    print("ngeom:", model.ngeom)


if __name__ == "__main__":
    main()
