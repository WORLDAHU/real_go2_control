from pathlib import Path

import mujoco


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PROJECT_ROOT / "models" / "single_leg" / "RL_single_leg_slider.urdf"


def main():
    print("URDF:", URDF_PATH)
    print("exists:", URDF_PATH.exists())
    print()

    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))

    print("MuJoCo loaded successfully")
    print("nq:", model.nq)
    print("nv:", model.nv)
    print("nu:", model.nu)
    print("njnt:", model.njnt)
    print("nbody:", model.nbody)
    print("ngeom:", model.ngeom)

    print()
    print("joints:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(" ", i, name)


if __name__ == "__main__":
    main()
