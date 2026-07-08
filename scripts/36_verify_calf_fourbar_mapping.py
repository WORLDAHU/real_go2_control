#!/usr/bin/env python3
from pathlib import Path
import sys


THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parents[1] if THIS_FILE.parent.name == "scripts" else THIS_FILE.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.real_leg_adapter import RealLegCommandAdapter, angle_error_deg


def fmt(value):
    if value is None:
        return "   None "
    return f"{value:8.2f}"


def main():
    adapter = RealLegCommandAdapter()
    cfg = adapter.fourbar

    print("Four-bar calf mapping check")
    print(
        "crank = crank_home + calf_motor_cmd * crank_deg_per_motor_deg "
        f"= {cfg.crank_home_deg:.2f} + cmd * {cfg.crank_deg_per_motor_deg:.3f}"
    )
    print(
        "calf/knee angle = wrap180(rocker_angle + "
        f"{cfg.calf_angle_offset_from_rocker_deg:.2f})"
    )
    print(
        "bridge command limit: "
        f"[{cfg.calf_motor_cmd_min_deg:.2f}, {cfg.calf_motor_cmd_max_deg:.2f}] deg"
    )

    print("\ncalf_motor_cmd -> crank_angle -> rocker_angle -> knee_pitch")
    print(" motor_cmd    crank   rocker     knee")
    for motor_cmd in [0.0, -10.0, -20.0, -40.0, -80.0, -120.0, -140.0]:
        crank = adapter.calf_motor_to_crank_angle(motor_cmd)
        rocker = adapter.crank_angle_to_rocker_angle(crank)
        knee = adapter.calf_motor_to_knee_pitch(motor_cmd)
        print(f"{motor_cmd:10.2f} {fmt(crank)} {fmt(rocker)} {fmt(knee)}")

    print("\nknee_pitch -> rocker_target -> crank_angle -> calf_motor_cmd -> knee_back")
    print("     knee   rocker    crank motor_cmd  clamped knee_back      err")
    for knee in [-160.59, -156.0, -140.0, -120.0, -100.0, -90.0, -70.0, -48.0]:
        rocker_target = adapter.knee_pitch_to_rocker_angle(knee)
        crank = adapter.inverse_fourbar_crank_angle(rocker_target)
        motor_cmd = adapter.crank_angle_to_calf_motor(crank)
        motor_cmd_clamped = max(
            cfg.calf_motor_cmd_min_deg,
            min(cfg.calf_motor_cmd_max_deg, motor_cmd),
        )
        knee_back = adapter.calf_motor_to_knee_pitch(motor_cmd)
        err = angle_error_deg(knee_back, knee)
        print(
            f"{knee:9.2f} {fmt(rocker_target)} {fmt(crank)}"
            f" {fmt(motor_cmd)} {fmt(motor_cmd_clamped)} {fmt(knee_back)} {err:8.3f}"
        )


if __name__ == "__main__":
    main()
