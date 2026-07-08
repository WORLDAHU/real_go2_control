from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from jump_controller import JumpController


robot = RealGo2Model()
controller = JumpController(robot)

phases = ["prepare", "push", "flight", "landing"]

for phase in phases:
    foot_forces = controller.compute_phase_foot_forces(phase)
    tau = controller.compute_phase_torque(phase)

    print("=" * 60)
    print("phase:", phase)
    print()

    print("foot forces:")
    for leg, force in foot_forces.items():
        print(f"  {leg}: {force}")

    print()
    print("tau:")
    print(tau)

    print()
    print("tau shape:", tau.shape)