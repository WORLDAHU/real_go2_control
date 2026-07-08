import numpy as np

from leg_controller import LegController


class JumpController:
    """
    JumpController 用来生成一个简单的竖直起跳动作。

    当前版本只做力规划，不做仿真，也不直接控制真实机器人。

    起跳动作分成 4 个阶段:

    1. prepare
       站稳阶段，四条腿平均支撑身体重量。

    2. push
       蹬地阶段，四条腿产生更大的向上支撑力。

    3. flight
       腾空阶段，脚不接触地面，足端力为 0。

    4. landing
       落地缓冲阶段，四条腿产生较大的支撑力用于缓冲。
    """

    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.leg_controller = LegController(robot_model)

        self.gravity = 9.81

        # 这两个参数后面可以调。
        # push_acc 越大，蹬地越猛。
        # landing_acc 越大，落地缓冲越强。
        self.push_acc = 8.0
        self.landing_acc = 5.0
        # 起跳动作每个阶段持续时间，单位: 秒
        self.prepare_duration = 0.5
        self.push_duration = 0.2
        self.flight_duration = 0.3
        self.landing_duration = 0.2

        self.total_duration = (
            self.prepare_duration
            + self.push_duration
            + self.flight_duration
            + self.landing_duration
        )

    def get_robot_mass(self):
        """
        获取机器人总质量。
        """

        mass = 0.0

        for inertia in self.robot_model.model.inertias:
            mass += inertia.mass

        return mass

    def make_equal_vertical_forces(self, force_per_leg):
        """
        给四条腿生成相同的竖直方向足端力。

        force_per_leg:
            每条腿的 z 方向力，单位 N。
        """

        return {
            "FL": np.array([0.0, 0.0, force_per_leg]),
            "FR": np.array([0.0, 0.0, force_per_leg]),
            "RL": np.array([0.0, 0.0, force_per_leg]),
            "RR": np.array([0.0, 0.0, force_per_leg]),
        }

    def compute_phase_foot_forces(self, phase):
        """
        根据起跳阶段，计算四条腿足端力。

        phase 可以是:
        - "prepare"
        - "push"
        - "flight"
        - "landing"
        """

        mass = self.get_robot_mass()

        if phase == "prepare":
            # 只抵消重力，保持站立
            force_per_leg = mass * self.gravity / 4.0

        elif phase == "push":
            # 抵消重力 + 额外向上加速度
            force_per_leg = mass * (self.gravity + self.push_acc) / 4.0

        elif phase == "flight":
            # 腾空时脚不接触地面
            force_per_leg = 0.0

        elif phase == "landing":
            # 落地缓冲，给一个比站立更大的支撑力
            force_per_leg = mass * (self.gravity + self.landing_acc) / 4.0

        else:
            raise ValueError(f"未知起跳阶段: {phase}")

        return self.make_equal_vertical_forces(force_per_leg)

    def compute_phase_torque(self, phase):
        """
        根据起跳阶段，计算 12 个关节力矩。
        """

        foot_forces = self.compute_phase_foot_forces(phase)

        tau = self.leg_controller.compute_all_joint_torques_from_foot_forces(
            foot_forces
        )

        return tau
    def get_phase_at_time(self, t):
        """
        根据当前时间 t 判断起跳阶段。

        t 单位是秒，从动作开始计时。
        """

        if t < 0:
            raise ValueError("时间 t 不能小于 0")

        prepare_end = self.prepare_duration
        push_end = prepare_end + self.push_duration
        flight_end = push_end + self.flight_duration
        landing_end = flight_end + self.landing_duration

        if t < prepare_end:
            return "prepare"
        elif t < push_end:
            return "push"
        elif t < flight_end:
            return "flight"
        elif t < landing_end:
            return "landing"
        else:
            return "done"

    def compute_torque_at_time(self, t):
        """
        根据当前时间 t 计算关节力矩。

        如果动作已经结束，返回 12 个 0。
        """

        phase = self.get_phase_at_time(t)

        if phase == "done":
            return phase, np.zeros(12)

        tau = self.compute_phase_torque(phase)

        return phase, tau
    def smoothstep(self, s):
        """
        平滑插值函数。

        输入 s:
            0 到 1 之间

        输出:
            也是 0 到 1 之间，但变化更平滑。

        公式:
            3s^2 - 2s^3

        特点:
            s=0 时输出 0
            s=1 时输出 1
            开始和结束时斜率都是 0
        """

        s = np.clip(s, 0.0, 1.0)
        return 3.0 * s**2 - 2.0 * s**3

    def get_force_per_leg_at_time(self, t):
        """
        根据当前时间 t，计算每条腿的竖直支撑力。

        和 compute_phase_foot_forces 不同，这个函数会让力平滑变化。
        """

        phase = self.get_phase_at_time(t)
        mass = self.get_robot_mass()

        stand_force = mass * self.gravity / 4.0
        push_force = mass * (self.gravity + self.push_acc) / 4.0
        landing_force = mass * (self.gravity + self.landing_acc) / 4.0

        prepare_end = self.prepare_duration
        push_end = prepare_end + self.push_duration
        flight_end = push_end + self.flight_duration
        landing_end = flight_end + self.landing_duration

        if phase == "prepare":
            return stand_force

        if phase == "push":
            # push 阶段从站立力平滑增加到蹬地力
            s = (t - prepare_end) / self.push_duration
            alpha = self.smoothstep(s)
            return (1.0 - alpha) * stand_force + alpha * push_force

        if phase == "flight":
            return 0.0

        if phase == "landing":
            # landing 阶段从较大缓冲力平滑回到站立力
            s = (t - flight_end) / self.landing_duration
            alpha = self.smoothstep(s)
            return (1.0 - alpha) * landing_force + alpha * stand_force

        if phase == "done":
            return 0.0

        raise ValueError(f"未知阶段: {phase}")

    def compute_smooth_torque_at_time(self, t):
        """
        根据当前时间 t，计算平滑版本的关节力矩。
        """

        phase = self.get_phase_at_time(t)
        force_per_leg = self.get_force_per_leg_at_time(t)
        foot_forces = self.make_equal_vertical_forces(force_per_leg)

        tau = self.leg_controller.compute_all_joint_torques_from_foot_forces(
            foot_forces
        )

        return phase, force_per_leg, tau