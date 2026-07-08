from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PROJECT_ROOT / "models" / "custom_robot" / "GO2.urdf"
BACKUP_PATH = PROJECT_ROOT / "models" / "custom_robot" / "GO2_before_mujoco_fix.urdf"


def is_zero_collision(collision):
    box = collision.find("./geometry/box")
    if box is not None and box.attrib.get("size") == "0 0 0":
        return True

    cylinder = collision.find("./geometry/cylinder")
    if cylinder is not None:
        radius = float(cylinder.attrib.get("radius", "0"))
        length = float(cylinder.attrib.get("length", "0"))
        if radius <= 0.0 or length <= 0.0:
            return True

    sphere = collision.find("./geometry/sphere")
    if sphere is not None:
        radius = float(sphere.attrib.get("radius", "0"))
        if radius <= 0.0:
            return True

    return False


if not BACKUP_PATH.exists():
    BACKUP_PATH.write_text(URDF_PATH.read_text())

tree = ET.parse(URDF_PATH)
root = tree.getroot()

removed = 0

for link in root.findall("link"):
    for collision in list(link.findall("collision")):
        if is_zero_collision(collision):
            link.remove(collision)
            removed += 1

ET.indent(tree, space="  ")
tree.write(URDF_PATH, encoding="utf-8", xml_declaration=True)

print("URDF:", URDF_PATH)
print("Backup:", BACKUP_PATH)
print("Removed zero-size collision blocks:", removed)