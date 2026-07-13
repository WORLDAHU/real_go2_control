from pathlib import Path
import re

urdf_path = Path(__file__).resolve().parents[1] / "models" / "custom_robot" / "GO2.urdf"

text = urdf_path.read_text()

counter = 0

def add_name(match):
    global counter
    counter += 1
    return f'<material name="auto_material_{counter:02d}">'

text = re.sub(r"<material>", add_name, text)

urdf_path.write_text(text)

print(f"Updated {counter} material tags in {urdf_path}")