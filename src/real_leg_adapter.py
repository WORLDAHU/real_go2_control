
"""
实机单腿角度适配模块。

这个文件只负责“角度名字和角度层级转换”，不直接控制电机，也不直接访问串口。

当前角度分三层：

1. sim_joint_deg
   MuJoCo / URDF 里的仿真关节角，来自 SingleLegSliderController 的 q_des。
   顺序是：
     q_des[0] -> hip_joint   -> RL_hip_joint
     q_des[1] -> thigh_joint -> RL_thigh_joint
     q_des[2] -> calf_joint  -> RL_calf_joint

2. common_joint_deg
   我们自己定义的通用机械关节角，用来统一运动学、机械零位、方向和实机测试。
   这一层是以后讨论“腿怎么动”的主要坐标层。
     hip_abduction = 0 deg：腿不外摆/内收。
     thigh_pitch   = 0 deg：大腿竖直向下。
     knee_pitch    = 0 deg：小腿和大腿共线。
     knee_pitch 正方向：从 +Y 的位置看向 XZ 平面，逆时针为正。

3. bridge_cmd_deg
   准备发给实机 bridge 的电机角度命令。
     hip_motor   当前由 hip_abduction 直接得到。
     thigh_motor 当前由 thigh_pitch 直接得到。
     calf_motor  由 knee_pitch 经过四连杆反解得到。
     calf_motor_unclamped 是限幅前结果，用来检查四连杆映射是否异常。

注意：
- hip/thigh 后续如果发现实机零位或方向不同，应该改 sim_zero_deg 或 direction。
- calf_motor 不能直接等于 calf_joint，也不能直接等于 knee_pitch，因为实机小腿有四连杆。
"""






import math
from dataclasses import dataclass


# 单关节的 sim -> common 映射配置。
# 公式是：
#   common = direction * (sim_deg - sim_zero_deg)
#
# sim_zero_deg:
#   common 机械零位在仿真角度里对应的角度。
#
# direction:
#   仿真正方向和 common 正方向一致时为 1，相反时为 -1。
#
# min_deg / max_deg:
#   common 层软限幅，防止后续发给实机的目标角过大。
@dataclass
class SimToCommonJoint:
    name: str
    sim_zero_deg: float
    direction: float
    min_deg: float
    max_deg: float

    def convert(self, sim_deg):
        common = self.direction * (sim_deg - self.sim_zero_deg)
        return max(self.min_deg, min(self.max_deg, common))



# 小腿四连杆参数。
#
# 仿真里的 calf_joint 是等效膝关节角；
# 实机里的 calf_motor 是四连杆输入电机角。
#
# 所以小腿角度链路是：
#   calf_joint -> knee_pitch -> 四连杆反解 -> calf_motor
#
# 不能直接写：
#   calf_motor = calf_joint
@dataclass
class FourBarConfig:
    crank_mm: float = 35.7#实际是35.7，有误差
    coupler_mm: float = 150.0
    rocker_mm: float = 30.0
    ground_mm: float = 164.0
    knee_offset_deg: float = 28.0
    motor2_zero_trim_deg: float = 18.0
    calf_motor_min_deg: float = 0.0
    calf_motor_max_deg: float = 165.0


class RealLegCommandAdapter:
    """
    三层角度适配：

    1. sim_joint_deg：
       MuJoCo / URDF 里的仿真关节角，直接来自当前单腿模型。
       hip_joint   = RL_hip_joint
       thigh_joint = RL_thigh_joint
       calf_joint  = RL_calf_joint

    2. common_joint_deg：
       我们自己定义的通用机械关节角，用来统一运动学、实机零位和方向。
       hip_abduction = 0 deg：腿不外摆/内收。
       thigh_pitch   = 0 deg：大腿竖直向下。
       knee_pitch    = 0 deg：小腿和大腿共线。
       knee_pitch 正方向：从 +Y 的位置看向 XZ 平面，逆时针为正。

    3. bridge_cmd_deg：
       发给实机 bridge 的电机角度命令。
       hip_motor   由 hip_abduction 直接得到，后续可加零位/方向修正。
       thigh_motor 由 thigh_pitch 直接得到，后续可加零位/方向修正。
       calf_motor  由 knee_pitch 经过四连杆反解得到。
       calf_motor_unclamped 表示限幅前的小腿电机角，用来调试四连杆映射是否异常。
    """
    def __init__(self):
        self.hip_abduction_common = SimToCommonJoint(
            name="hip_abduction",
            sim_zero_deg=0.0,
            direction=1.0,
            min_deg=-45.0,
            max_deg=45.0,
        )

        self.thigh_pitch_common = SimToCommonJoint(
            name="thigh_pitch",
            sim_zero_deg=0.0,
            direction=1.0,
            min_deg=-90.0,
            max_deg=90.0,
        )

        self.knee_pitch_common = SimToCommonJoint(
            name="knee_pitch",
            sim_zero_deg=0.0,
            direction=-1.0,
            min_deg=0.0,
            max_deg=160.0,
        )

        self.fourbar = FourBarConfig()

        self.motor_config = {
            "hip_motor": {"port": "/dev/ttyUSB0", "id": 2, "dir": 1},
            "thigh_motor": {"port": "/dev/ttyUSB2", "id": 0, "dir": 1},
            "calf_motor": {"port": "/dev/ttyUSB3", "id": 1, "dir": -1},
        }

    def q_des_to_command(self, q_des_rad):
        # q_des_rad 是控制器输出的目标仿真关节角，单位 rad。
        # 这里按三层顺序转换：
        #   1. rad -> sim_joint_deg
        #   2. sim_joint_deg -> common_joint_deg
        #   3. common_joint_deg -> bridge_cmd_deg
        sim_joint_deg = self.sim_rad_to_sim_joint_deg(q_des_rad)
        common_joint_deg = self.sim_to_common_joint_deg(sim_joint_deg)
        bridge_cmd_deg = self.common_to_bridge_cmd_deg(common_joint_deg)

        return {
            "sim_joint_deg": sim_joint_deg,
            "common_joint_deg": common_joint_deg,
            "bridge_cmd_deg": bridge_cmd_deg,
        }

    def sim_rad_to_sim_joint_deg(self, q_rad):
        return {
            "hip_joint": math.degrees(q_rad[0]),
            "thigh_joint": math.degrees(q_rad[1]),
            "calf_joint": math.degrees(q_rad[2]),
        }

    def sim_to_common_joint_deg(self, sim_joint_deg):
        return {
            "hip_abduction": self.hip_abduction_common.convert(
                sim_joint_deg["hip_joint"]
            ),
            "thigh_pitch": self.thigh_pitch_common.convert(
                sim_joint_deg["thigh_joint"]
            ),
            "knee_pitch": self.knee_pitch_common.convert(
                sim_joint_deg["calf_joint"]
            ),
        }

    def common_to_bridge_cmd_deg(self, common_joint_deg):
        hip_motor = common_joint_deg["hip_abduction"]
        thigh_motor = common_joint_deg["thigh_pitch"]

        calf_motor_unclamped = self.knee_pitch_to_calf_motor(
            common_joint_deg["knee_pitch"]
        )
        calf_motor = max(
            self.fourbar.calf_motor_min_deg,
            min(self.fourbar.calf_motor_max_deg, calf_motor_unclamped),
        )

        return {
            "hip_motor": hip_motor,
            "thigh_motor": thigh_motor,
            "calf_motor": calf_motor,
            "calf_motor_unclamped": calf_motor_unclamped,
        }

    def knee_pitch_to_calf_motor(self, knee_pitch_deg):
        # knee_pitch 是 common 层的等效膝关节角。
        # 实机小腿电机不是膝关节本身，而是四连杆输入端。
        # 因此这里先把 knee_pitch 转成四连杆摇杆目标 beta_des，
        # 再反解输入曲柄 alpha，最后得到 calf_motor。
        beta_des = knee_pitch_deg - self.fourbar.knee_offset_deg
        alpha = self.inverse_fourbar_alpha(beta_des)
        return -2.0 * alpha - self.fourbar.motor2_zero_trim_deg

    def inverse_fourbar_alpha(self, beta_des_deg):
        best_alpha = None
        best_err = 1e9

        alpha_min = -(
            self.fourbar.calf_motor_max_deg + self.fourbar.motor2_zero_trim_deg
        ) / 2.0
        alpha_max = -(
            self.fourbar.calf_motor_min_deg + self.fourbar.motor2_zero_trim_deg
        ) / 2.0

        for i in range(1001):
            alpha = alpha_min + (alpha_max - alpha_min) * i / 1000.0
            beta = self.fourbar_beta(alpha)
            if beta is None:
                continue

            err = abs(beta - beta_des_deg)
            err = min(err, 360.0 - err)

            if err < best_err:
                best_err = err
                best_alpha = alpha

        if best_alpha is None:
            return alpha_max

        return best_alpha

    def fourbar_beta(self, alpha_deg):
        cfg = self.fourbar

        k1 = cfg.ground_mm / cfg.crank_mm
        k2 = -cfg.ground_mm / cfg.rocker_mm
        k3 = (
            cfg.crank_mm**2
            - cfg.coupler_mm**2
            + cfg.rocker_mm**2
            + cfg.ground_mm**2
        ) / (2.0 * cfg.crank_mm * cfg.rocker_mm)

        alpha = math.radians(alpha_deg)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        a = ca - k1
        b = sa
        c = -(k2 * ca + k3)

        disc = a * a + b * b - c * c
        if disc < 0.0:
            return None

        beta_raw = 2.0 * math.atan2(-b - math.sqrt(disc), c - a)
        beta_user = 180.0 + math.degrees(beta_raw)
        return beta_user % 360.0
