import numpy as np

from leg_controller import LegController


class StandingController:
    """
    简单站立控制器。

    目标：
    让四条腿平均承担机器人重力。

    当前版本没有考虑姿态误差、速度误差、落脚点等复杂因素，
    只做最基础的力分配：

        每条腿支撑力 = 机器人总重量 / 4
    """

    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.leg_controller = LegController(robot_model)

    def get_robot_mass(self):
        """
        从 Pinocchio 模型中读取机器人总质量。
        """

        mass = 0.0

        # model.inertias 里保存每个 link 的质量和惯量
        for inertia in self.robot_model.model.inertias:
            mass += inertia.mass

        return mass

    def compute_standing_foot_forces(self):
        """
        计算站立时四条腿的足端支撑力。
        """

        mass = self.get_robot_mass()
        gravity = 9.81

        total_weight = mass * gravity
        force_per_leg = total_weight / 4.0

        # 只给 z 方向向上的力
        return {
            "FL": np.array([0.0, 0.0, force_per_leg]),
            "FR": np.array([0.0, 0.0, force_per_leg]),
            "RL": np.array([0.0, 0.0, force_per_leg]),
            "RR": np.array([0.0, 0.0, force_per_leg]),
        }

    def compute_standing_torque(self):
        """
        计算站立所需的 12 个关节力矩。
        """

        foot_forces_world = self.compute_standing_foot_forces()

        return self.leg_controller.compute_all_joint_torques_from_foot_forces(
            foot_forces_world
        )