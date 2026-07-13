from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from jump_controller import JumpController


robot = RealGo2Model()
controller = JumpController(robot)

dt = 0.1
times = np.arange(0.0, controller.total_duration + 0.3, dt)

print("jump total duration:", controller.total_duration)
print()

for t in times:
    phase, tau = controller.compute_torque_at_time(t)

    print(f"t={t:.2f}s  phase={phase:8s}  tau_norm={np.linalg.norm(tau):.3f}")