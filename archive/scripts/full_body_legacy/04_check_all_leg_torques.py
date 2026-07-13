from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from leg_controller import LegController


robot = RealGo2Model()
controller = LegController(robot)

# 假设四条腿都在支撑，每条腿获得 20N 向上支撑力
foot_forces_world = {
    "FL": np.array([0.0, 0.0, 20.0]),
    "FR": np.array([0.0, 0.0, 20.0]),
    "RL": np.array([0.0, 0.0, 20.0]),
    "RR": np.array([0.0, 0.0, 20.0]),
}

all_tau = controller.compute_all_joint_torques_from_foot_forces(foot_forces_world)

print("12 个关节力矩:")
print(all_tau)
print()
print("形状:", all_tau.shape)

print()
print("按腿拆开看:")
print("FL:", all_tau[0:3])
print("FR:", all_tau[3:6])
print("RL:", all_tau[6:9])
print("RR:", all_tau[9:12])