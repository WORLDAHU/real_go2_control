import numpy as np


class LegController:
    """
    腿部控制器。

    当前这个类只做一件核心事情：

        足端力 -> 关节力矩

    使用的公式是：

        tau = J.T @ force

    其中：
    - J 是足端 Jacobian，描述关节速度和足端速度的关系
    - force 是期望地面对脚的力
    - tau 是需要施加到关节上的力矩
    """

    def __init__(self, robot_model):
        # robot_model 是 RealGo2Model 对象
        # 它提供 get_foot_jacobian(leg) 等运动学函数
        self.robot_model = robot_model

    def compute_joint_torque_from_foot_force(self, leg, force_world):
        """
        根据某条腿的足端力，计算该腿 3 个关节的力矩。

        参数：
        leg:
            腿名，取值为 "FL", "FR", "RL", "RR"

        force_world:
            世界坐标系下的足端力 [fx, fy, fz]
            可以理解为“地面对脚的反作用力”

        返回：
        tau:
            该腿 3 个关节的力矩 [hip, thigh, calf]
        """

        # 确保输入是长度为 3 的 numpy 数组
        force_world = np.asarray(force_world, dtype=float).reshape(3)

        # 获取这条腿的 3x3 足端 Jacobian
        jacobian = self.robot_model.get_foot_jacobian(leg)

        # Jacobian 转置把足端力映射到关节力矩
        tau = jacobian.T @ force_world

        return tau

    def compute_all_joint_torques_from_foot_forces(self, foot_forces_world):
        """
        根据四条腿的足端力，计算 12 个关节力矩。

        foot_forces_world 格式：

        {
            "FL": np.array([fx, fy, fz]),
            "FR": np.array([fx, fy, fz]),
            "RL": np.array([fx, fy, fz]),
            "RR": np.array([fx, fy, fz]),
        }

        返回顺序：
        FL hip/thigh/calf
        FR hip/thigh/calf
        RL hip/thigh/calf
        RR hip/thigh/calf
        """

        tau_list = []

        # 按固定顺序计算每条腿的 3 个力矩
        for leg in ["FL", "FR", "RL", "RR"]:
            tau_leg = self.compute_joint_torque_from_foot_force(
                leg,
                foot_forces_world[leg],
            )
            tau_list.append(tau_leg)

        # 4 个长度为 3 的数组拼成长度为 12 的数组
        return np.concatenate(tau_list)