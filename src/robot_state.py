import numpy as np


class RobotState:
    """
    RobotState 用来保存机器人当前状态。

    它主要管理两类量:

    1. q  = 广义位置
       包括:
       - 机身位置 base_pos
       - 机身姿态 base_quat
       - 12 个关节角

    2. dq = 广义速度
       包括:
       - 机身线速度 base_vel
       - 机身角速度 base_ang_vel
       - 12 个关节速度

    对 free-flyer 四足机器人来说:

    q  长度是 19:
       3 个 base 位置
       4 个 base 四元数
       12 个关节角

    dq 长度是 18:
       3 个 base 线速度
       3 个 base 角速度
       12 个关节速度
    """

    def __init__(self):
        # =========================
        # 机身初始状态
        # =========================

        # 机身位置 [x, y, z]，单位: 米
        self.base_pos = np.array([0.0, 0.0, 0.27])

        # 机身姿态四元数 [x, y, z, w]
        # [0, 0, 0, 1] 表示没有旋转
        self.base_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # 机身线速度 [vx, vy, vz]
        self.base_vel = np.array([0.0, 0.0, 0.0])

        # 机身角速度 [wx, wy, wz]
        self.base_ang_vel = np.array([0.0, 0.0, 0.0])

        # =========================
        # 四条腿初始关节角
        # =========================

        # 每条腿顺序: hip, thigh, calf
        self.FL_joint_angle = np.array([0.0, 0.9, -1.8])
        self.FR_joint_angle = np.array([0.0, 0.9, -1.8])
        self.RL_joint_angle = np.array([0.0, 0.9, -1.8])
        self.RR_joint_angle = np.array([0.0, 0.9, -1.8])

        # =========================
        # 四条腿初始关节速度
        # =========================

        self.FL_joint_vel = np.array([0.0, 0.0, 0.0])
        self.FR_joint_vel = np.array([0.0, 0.0, 0.0])
        self.RL_joint_vel = np.array([0.0, 0.0, 0.0])
        self.RR_joint_vel = np.array([0.0, 0.0, 0.0])

    def get_q(self):
        """
        把当前状态拼成 Pinocchio 需要的 q。

        q 的顺序必须和 URDF 里关节顺序一致:

        base_pos
        base_quat
        FL joints
        FR joints
        RL joints
        RR joints
        """

        return np.concatenate([
            self.base_pos,
            self.base_quat,
            self.FL_joint_angle,
            self.FR_joint_angle,
            self.RL_joint_angle,
            self.RR_joint_angle,
        ])

    def get_dq(self):
        """
        把当前速度拼成 Pinocchio 需要的 dq。

        dq 的顺序:

        base_vel
        base_ang_vel
        FL joint velocities
        FR joint velocities
        RL joint velocities
        RR joint velocities
        """

        return np.concatenate([
            self.base_vel,
            self.base_ang_vel,
            self.FL_joint_vel,
            self.FR_joint_vel,
            self.RL_joint_vel,
            self.RR_joint_vel,
        ])

    def update_from_q_dq(self, q, dq):
        """
        用一组 q 和 dq 更新 RobotState。

        以后我们从仿真器或者真实机器人读到状态后，
        可以用这个函数把状态拆回更好读的字段。
        """

        # q 前 7 个是机身位置和姿态
        self.base_pos = q[0:3]
        self.base_quat = q[3:7]

        # q 后 12 个是关节角
        joint_angles = q[7:19]
        self.FL_joint_angle = joint_angles[0:3]
        self.FR_joint_angle = joint_angles[3:6]
        self.RL_joint_angle = joint_angles[6:9]
        self.RR_joint_angle = joint_angles[9:12]

        # dq 前 6 个是机身线速度和角速度
        self.base_vel = dq[0:3]
        self.base_ang_vel = dq[3:6]

        # dq 后 12 个是关节速度
        joint_velocities = dq[6:18]
        self.FL_joint_vel = joint_velocities[0:3]
        self.FR_joint_vel = joint_velocities[3:6]
        self.RL_joint_vel = joint_velocities[6:9]
        self.RR_joint_vel = joint_velocities[9:12]