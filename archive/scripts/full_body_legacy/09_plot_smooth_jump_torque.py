from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model
from jump_controller import JumpController


robot = RealGo2Model()
controller = JumpController(robot)

dt = 0.01
times = np.arange(0.0, controller.total_duration + 0.2, dt)

tau_log = []
force_log = []
phase_log = []

for t in times:
    phase, force_per_leg, tau = controller.compute_smooth_torque_at_time(t)
    phase_log.append(phase)
    force_log.append(force_per_leg)
    tau_log.append(tau)

tau_log = np.array(tau_log)
force_log = np.array(force_log)

joint_names = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]

plt.figure(figsize=(12, 10))

plt.subplot(2, 1, 1)
plt.plot(times, force_log, linewidth=2)
plt.title("Smooth Jump Force Per Leg")
plt.ylabel("vertical force [N]")
plt.grid(True)

plt.axvline(controller.prepare_duration, color="black", linestyle="--", linewidth=1)
plt.axvline(controller.prepare_duration + controller.push_duration, color="black", linestyle="--", linewidth=1)
plt.axvline(
    controller.prepare_duration + controller.push_duration + controller.flight_duration,
    color="black",
    linestyle="--",
    linewidth=1,
)
plt.axvline(controller.total_duration, color="black", linestyle="--", linewidth=1)

plt.subplot(2, 1, 2)
for i in range(12):
    plt.plot(times, tau_log[:, i], label=joint_names[i])

plt.xlabel("time [s]")
plt.ylabel("joint torque [Nm]")
plt.grid(True)
plt.legend(ncol=3)

plt.axvline(controller.prepare_duration, color="black", linestyle="--", linewidth=1)
plt.axvline(controller.prepare_duration + controller.push_duration, color="black", linestyle="--", linewidth=1)
plt.axvline(
    controller.prepare_duration + controller.push_duration + controller.flight_duration,
    color="black",
    linestyle="--",
    linewidth=1,
)
plt.axvline(controller.total_duration, color="black", linestyle="--", linewidth=1)

plt.tight_layout()

output_path = PROJECT_ROOT / "smooth_jump_torque_timeline.png"
plt.savefig(output_path, dpi=200)

print("Saved plot to:", output_path)