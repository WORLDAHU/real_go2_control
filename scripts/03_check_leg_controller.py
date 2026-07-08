from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from leg_controller import LegController


robot = RealGo2Model()
controller = LegController(robot)

# 给每条腿一个向上的足端力。
# 这里先随便用 20N，只是为了检查计算链路。
force_world = np.array([0.0, 0.0, 20.0])

for leg in ["FL", "FR", "RL", "RR"]:
    tau = controller.compute_joint_torque_from_foot_force(leg, force_world)
    print(f"{leg} force:", force_world)
    print(f"{leg} torque:", tau)
    print()