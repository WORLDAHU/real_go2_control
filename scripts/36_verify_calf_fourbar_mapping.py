"""
小腿四连杆角度映射验证脚本。

这个脚本只做数学验证，不控制电机，不访问串口。
用于检查 real_leg_adapter.py 里当前小腿机构定义是否符合物理直觉和实机约定。

当前验证的物理链路是：

  正运动学：
    calf_motor_cmd_deg
      -> crank_angle_deg
      -> rocker_angle_deg
      -> knee_pitch

  反运动学：
    knee_pitch
      -> rocker_target_deg
      -> crank_angle_deg
      -> calf_motor_cmd_deg

当前角度定义：

1. calf_motor_cmd_deg
   发给实机 bridge 的小腿电机命令角。
   它是相对真实小腿电机零位的角度。

   当前约定：
     0 deg    表示真实电机零位。
     负角度   表示实机小腿可动方向。
     正角度   当前不允许用于实机动作。

   bridge 最终限幅为：
     [-140, 0] deg

2. crank_angle_deg
   四连杆曲柄角。
   曲柄与大齿轮固连。
   角度基准是机架方向：从曲柄转轴指向摇杆转轴的射线。

   当前模型：
     calf_motor_cmd_deg = 0
       -> crank_angle_deg = 10 deg

   当前齿轮/方向关系：
     crank_angle_deg = crank_home_deg
                       + calf_motor_cmd_deg * crank_deg_per_motor_deg

   其中：
     crank_home_deg = 10.0
     crank_deg_per_motor_deg = -0.5

   因此：
     calf_motor_cmd_deg 变负
       -> crank_angle_deg 增大

3. rocker_angle_deg
   四连杆摇杆角。
   角度基准与曲柄角定义平行，都是相对于机架方向。
   它由四连杆几何根据 crank_angle_deg 正运动学计算得到。

   当前几何尺寸：
     crank_mm  = 35.7
     coupler_mm = 150.0
     rocker_mm = 30.0
     ground_mm = 164.0

   当前模型下：
     crank_angle_deg = 10 deg
       -> rocker_angle_deg ≈ 47.4 deg

4. knee_pitch
   最终小腿机械角，也对应 common 层的 knee_pitch。
   它使用 GO2 小腿角度约定：
     膝盖弯曲为负数。

   摇杆和小腿固连，但不共线。
   当前物理模型中摇杆和小腿的夹角为 28 deg，
   按当前摇杆角方向定义，小腿角为：
     knee_pitch = wrap180(rocker_angle_deg + 180 - 28)
                = wrap180(rocker_angle_deg + 152)

   因此在电机零位时：
     calf_motor_cmd_deg = 0
       -> crank_angle_deg ≈ 10 deg
       -> rocker_angle_deg ≈ 47.4 deg
       -> knee_pitch ≈ wrap180(47.4 + 152)
       -> knee_pitch ≈ -160.6 deg

脚本输出两张表：

1. calf_motor_cmd -> crank -> rocker -> knee
   用于检查正运动学趋势：
     电机命令越负，曲柄角越大，摇杆角越大，膝盖角逐渐增大。

2. knee -> rocker_target -> crank -> motor_cmd -> knee_back
   用于检查反解是否能回到目标 knee_pitch。
   其中 motor_cmd 是反解得到的理论命令，
   clamped 是经过 bridge 命令限幅 [-140, 0] 后的实际可发送命令。现在改了

需要特别关注：
   如果 motor_cmd 小于 -140 deg，说明目标 knee_pitch 超出了当前实机安全命令范围。
   这时 bridge 会把命令夹到 -140 deg，真实小腿不能达到该目标角。
"""

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
