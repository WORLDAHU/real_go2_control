from pathlib import Path

import mujoco as mj


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PROJECT_ROOT / "models" / "custom_robot" / "GO2.urdf"


print("URDF path:", URDF_PATH)
print("exists:", URDF_PATH.exists())
print()

model = mj.MjModel.from_xml_path(str(URDF_PATH))
data = mj.MjData(model)

print("MuJoCo model loaded successfully")
print()
print("nq:", model.nq)
print("nv:", model.nv)
print("nu:", model.nu)
print("nbody:", model.nbody)
print("njnt:", model.njnt)
print("ngeom:", model.ngeom)
print()

print("joint names:")
for i in range(model.njnt):
    print(" ", i, mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i))

print()
print("actuator names:")
for i in range(model.nu):
    print(" ", i, mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i))