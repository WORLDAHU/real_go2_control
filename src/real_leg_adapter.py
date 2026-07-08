"""
实机单腿角度适配模块。

这个文件只负责角度层级转换，不直接控制电机，也不直接访问串口。

当前角度分三层：

1. sim_joint_deg
   MuJoCo / URDF 里的仿真关节角，来自单腿控制器输出的 q_des。
   当前顺序是：
     q_des[0] -> hip_joint   -> RL_hip_joint
     q_des[1] -> thigh_joint -> RL_thigh_joint
     q_des[2] -> calf_joint  -> RL_calf_joint

   其中 RL_calf_joint 使用 GO2 官方小腿关节角约定：
     膝盖弯曲方向为负数。
   例如：
     sim calf_joint = -120 deg 表示膝盖处于弯曲状态。

2. common_joint_deg
   我们自己统一讨论用的机械关节角层。
   这一层用于屏蔽仿真命名、实机零位、电机方向等差异，方便之后讨论机械状态。

   当前定义：
     hip_abduction = 0 deg：髋外展处于中位。
     thigh_pitch   = 0 deg：大腿处于约定机械零位。
     knee_pitch    = 0 deg：小腿和大腿共线。

   knee_pitch 采用 GO2 小腿关节约定：
     膝盖弯曲为负数。
   因此当前：
     common knee_pitch = sim RL_calf_joint
     direction = 1.0

   例如：
     sim RL_calf_joint = -120 deg
     common knee_pitch = -120 deg

3. bridge_cmd_deg
   准备发给实机 bridge 的电机命令角。

   hip_motor / thigh_motor 当前暂时由 common 关节角直接得到。
   calf_motor 不能直接等于 knee_pitch，因为真实小腿不是单一转轴直驱，
   中间包含电机、小齿轮/大齿轮、曲柄、连杆、摇杆和小腿固连关系。

   当前小腿物理链路定义为：
     calf_motor_cmd_deg -> crank_angle_deg -> rocker_angle_deg -> knee_pitch

   各变量含义：
     calf_motor_cmd_deg:
       发给 bridge 的真实小腿电机命令角。
       它是相对真实小腿电机零位的角度。
       当前实机可动方向是负方向，有效范围约为 [-180, 0] deg。

     crank_angle_deg:
       四连杆曲柄角。
       曲柄与大齿轮固连。
       角度基准是：从曲柄转轴指向摇杆转轴的机架射线。
       当前电机命令为 0 deg 时，曲柄角由物理模型决定为 10 deg。

     rocker_angle_deg:
       四连杆摇杆角。
       角度基准与曲柄角平行，也是相对于机架方向定义。
       在 calf_motor_cmd_deg = 0 时，几何计算得到 rocker_angle_deg 约为 47.4 deg。

     knee_pitch:
       最终小腿/膝盖机械角。
       摇杆和小腿固连，但二者不共线。
       当前物理模型中二者夹角为 28 deg，因此：
         knee_pitch = wrap180(rocker_angle_deg + (180 - 28))
                    = wrap180(rocker_angle_deg + 152)

   当前零位关系：
     calf_motor_cmd_deg = 0
       -> crank_angle_deg ≈ 10 deg
       -> rocker_angle_deg ≈ 47.4 deg
       -> knee_pitch ≈ wrap180(47.4 + 152)
       -> knee_pitch ≈ -160.6 deg

   当前方向关系：
     calf_motor_cmd_deg 减小，也就是往负方向运动
       -> crank_angle_deg 增大
       -> rocker_angle_deg 增大
       -> knee_pitch 增大，但仍然通常保持为负数
       -> 膝盖从更弯曲状态逐渐伸开

   注意：
   - bridge 的 calf_motor 限幅 [-180, 0] 只用于最终实机命令保护。
   - 四连杆内部求解应该使用 crank_min/max 等几何角范围。
   - 不要把 bridge 命令范围直接拿去当四连杆内部曲柄角范围。
   - 旧的 motor2_zero_trim_deg 语义容易混合电机角、齿轮角和曲柄角，
     当前已拆成 crank_home_deg、crank_deg_per_motor_deg 等更明确的物理参数。
"""







import math
from dataclasses import dataclass


def wrap_deg_360(angle_deg):
    return angle_deg % 360.0


def wrap_deg_180(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def angle_error_deg(a_deg, b_deg):
    return abs(wrap_deg_180(a_deg - b_deg))


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
# 实机里的 calf_motor_cmd 是发给真实小腿电机的相对零位命令角。
#
# 这里把物理链路拆开命名：
#   calf_motor_cmd_deg  ->  crank_angle_deg
#   crank_angle_deg     ->  rocker_angle_deg
#   rocker_angle_deg    ->  calf/knee mechanical angle
#
# 不能直接写：
#   calf_motor_cmd_deg = calf_joint
@dataclass
class FourBarConfig:
    crank_mm: float = 35.7
    coupler_mm: float = 150.0
    rocker_mm: float = 30.0
    ground_mm: float = 164.0

    # Go2 small-calf convention: knee bending is negative.
    # At calf_motor_cmd_deg = 0, the measured/modelled calf angle is about -160.59 deg.
    knee_pitch_home_deg: float = -160.59

    # Four-bar crank angle. The reference ray points from the crank pivot to the
    # rocker pivot. At real motor zero, the big gear and crank are fixed here.
    crank_home_deg: float = 10.0
    crank_min_deg: float = 0.0
    crank_max_deg: float = 165.0

    # Gear/sign relation between the bridge command and the crank.
    # A negative motor command increases the crank angle:
    #   crank_angle = crank_home + calf_motor_cmd * crank_deg_per_motor_deg
    crank_deg_per_motor_deg: float = -0.5

    # Final bridge command relative to the real motor zero.
    # For the calf setup, the allowed motion goes from 0 toward negative angle.
    calf_motor_cmd_min_deg: float = -180.0
    calf_motor_cmd_max_deg: float = 0.0

    # The rocker and calf are fixed but not collinear. With the rocker angle's
    # positive direction, the calf angle is rocker + (180 - 28) deg.
    rocker_to_calf_inner_deg: float = 28.0

    @property
    def calf_angle_offset_from_rocker_deg(self):
        return 180.0 - self.rocker_to_calf_inner_deg

    # Backward-compatible aliases for older debug snippets.
    @property
    def knee_home_deg(self):
        return self.knee_pitch_home_deg

    @property
    def calf_motor_min_deg(self):
        return self.calf_motor_cmd_min_deg

    @property
    def calf_motor_max_deg(self):
        return self.calf_motor_cmd_max_deg


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
       knee_pitch    采用 Go2 小腿关节约定，膝盖弯曲为负数。
       常用范围约为 -156 deg 到 -48 deg。

    3. bridge_cmd_deg：
       发给实机 bridge 的电机角度命令。
       hip_motor   由 hip_abduction 直接得到，后续可加零位/方向修正。
       thigh_motor 由 thigh_pitch 直接得到，后续可加零位/方向修正。
       calf_motor  是相对真实小腿电机零位的命令角，经过四连杆反解得到。
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
            direction=1.0,
            min_deg=-160.59,
            max_deg=-48.0,
        )

        self.fourbar = FourBarConfig()

        self.motor_config = {
            "hip_motor": {"port": "/dev/ttyUSB0", "id": 2, "dir": 1},
            "thigh_motor": {"port": "/dev/ttyUSB2", "id": 0, "dir": 1},
            "calf_motor": {"port": "/dev/ttyUSB3", "id": 1, "dir": 1},
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
            self.fourbar.calf_motor_cmd_min_deg,
            min(self.fourbar.calf_motor_cmd_max_deg, calf_motor_unclamped),
        )

        return {
            "hip_motor": hip_motor,
            "thigh_motor": thigh_motor,
            "calf_motor": calf_motor,
            "calf_motor_unclamped": calf_motor_unclamped,
        }

    def knee_pitch_to_calf_motor(self, knee_pitch_deg):
        # common knee_pitch follows the Go2 convention:
        # knee bending is negative, e.g. -90 deg.
        #
        # The physical inverse chain is:
        #   knee_pitch -> rocker target -> crank target -> motor command.
        rocker_target_deg = self.knee_pitch_to_rocker_angle(knee_pitch_deg)
        crank_target_deg = self.inverse_fourbar_crank_angle(rocker_target_deg)
        return self.crank_angle_to_calf_motor(crank_target_deg)

    def calf_motor_to_knee_pitch(self, calf_motor_cmd_deg):
        # Positive direction is the bridge command convention:
        # command 0 is home; negative command increases the crank angle.
        crank_angle_deg = self.calf_motor_to_crank_angle(calf_motor_cmd_deg)
        rocker_angle_deg = self.crank_angle_to_rocker_angle(crank_angle_deg)
        if rocker_angle_deg is None:
            raise ValueError(
                f"crank angle out of four-bar reach: {crank_angle_deg:.3f} deg"
            )

        calf_angle_deg = (
            rocker_angle_deg + self.fourbar.calf_angle_offset_from_rocker_deg
        )
        return wrap_deg_180(calf_angle_deg)

    def calf_motor_to_crank_angle(self, calf_motor_cmd_deg):
        cfg = self.fourbar
        return cfg.crank_home_deg + calf_motor_cmd_deg * cfg.crank_deg_per_motor_deg

    def crank_angle_to_calf_motor(self, crank_angle_deg):
        cfg = self.fourbar
        if cfg.crank_deg_per_motor_deg == 0.0:
            raise ValueError("crank_deg_per_motor_deg cannot be zero")
        return (crank_angle_deg - cfg.crank_home_deg) / cfg.crank_deg_per_motor_deg

    def knee_pitch_to_rocker_angle(self, knee_pitch_deg):
        calf_angle_deg = wrap_deg_360(knee_pitch_deg)
        return wrap_deg_360(
            calf_angle_deg - self.fourbar.calf_angle_offset_from_rocker_deg
        )

    def inverse_fourbar_crank_angle(self, rocker_target_deg):
        best_crank = None
        best_err = 1e9

        crank_min = max(self.fourbar.crank_min_deg, self.fourbar.crank_home_deg)
        crank_max = self.fourbar.crank_max_deg

        steps = 3000
        for i in range(steps + 1):
            crank_angle = crank_min + (crank_max - crank_min) * i / steps
            rocker_angle = self.crank_angle_to_rocker_angle(crank_angle)
            if rocker_angle is None:
                continue

            err = angle_error_deg(rocker_angle, rocker_target_deg)

            if err < best_err:
                best_err = err
                best_crank = crank_angle

        if best_crank is None:
            return crank_min

        return best_crank

    def crank_angle_to_rocker_angle(self, crank_angle_deg):
        cfg = self.fourbar

        k1 = cfg.ground_mm / cfg.crank_mm
        k2 = -cfg.ground_mm / cfg.rocker_mm
        k3 = (
            cfg.crank_mm**2
            - cfg.coupler_mm**2
            + cfg.rocker_mm**2
            + cfg.ground_mm**2
        ) / (2.0 * cfg.crank_mm * cfg.rocker_mm)

        crank_angle = math.radians(crank_angle_deg)
        ca = math.cos(crank_angle)
        sa = math.sin(crank_angle)

        a = ca - k1
        b = sa
        c = -(k2 * ca + k3)

        disc = a * a + b * b - c * c
        if disc < 0.0:
            return None

        rocker_raw = 2.0 * math.atan2(-b + math.sqrt(disc), c - a)
        return wrap_deg_360(math.degrees(rocker_raw))

    # Backward-compatible aliases for older debug snippets.
    def inverse_fourbar_alpha(self, beta_des_deg):
        return self.inverse_fourbar_crank_angle(beta_des_deg)

    def fourbar_beta(self, alpha_deg):
        return self.crank_angle_to_rocker_angle(alpha_deg)
