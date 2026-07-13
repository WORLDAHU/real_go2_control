from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from standing_controller import StandingController


robot = RealGo2Model()
controller = StandingController(robot)

mass = controller.get_robot_mass()
foot_forces = controller.compute_standing_foot_forces()
tau = controller.compute_standing_torque()

print("机器人质量 mass:", mass)
print()
print("四条腿平均支撑力:")
for leg, force in foot_forces.items():
    print(f"{leg}: {force}")

print()
print("站立关节力矩 tau:")
print(tau)

print()
print("tau 形状:", tau.shape)