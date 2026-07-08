from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from robot_state import RobotState


# =========================
# 路径设置
# =========================

# 当前文件是:
# real_go2_control/src/real_go2_model.py
#
# parents[1] 表示向上两级，得到项目根目录:
# real_go2_control/
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# 机器人模型文件夹:
# real_go2_control/models/custom_robot/
ROBOT_DIR = PROJECT_ROOT / "models" / "custom_robot"

# URDF 文件路径:
# real_go2_control/models/custom_robot/GO2.urdf
URDF_PATH = ROBOT_DIR / "GO2.urdf"


# =========================
# 机器人命名约定
# =========================

# 四条腿的简称:
# FL = Front Left  左前腿
# FR = Front Right 右前腿
# RL = Rear Left   左后腿
# RR = Rear Right  右后腿
LEG_NAMES = ["FL", "FR", "RL", "RR"]


# 每条腿对应的三个主动关节。
# 顺序很重要，后面算 Jacobian、力矩控制时会用到。
#
# hip   髋关节，通常控制腿向内/向外摆
# thigh 大腿关节，通常控制腿向前/向后摆
# calf  小腿关节，通常控制膝盖弯曲
JOINT_NAMES = {
    "FL": ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"],
    "FR": ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"],
    "RL": ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"],
    "RR": ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
}


# 每条腿的足端 frame 名字。
# 这里用 fixed joint 作为足端位置参考点。
FOOT_FRAME_NAMES = {
    "FL": "FL_foot_joint",
    "FR": "FR_foot_joint",
    "RL": "RL_foot_joint",
    "RR": "RR_foot_joint",
}


class RealGo2Model:
    """
    RealGo2Model 是我们自己的机器人模型类。

    它目前负责三件事:
    1. 读取 GO2.urdf
    2. 生成 Pinocchio 里的 model 和 data
    3. 提供一些后续控制会用到的基础函数:
       - 足端位置
       - 足端 Jacobian
       - 初始站立姿态
    """

    def __init__(self):
        """
        初始化机器人模型。

        这一步会读取 URDF，并构造 Pinocchio 模型。
        如果 URDF 路径不对，会直接报错。
        """

        if not URDF_PATH.exists():
            raise FileNotFoundError(f"找不到 URDF 文件: {URDF_PATH}")

        # 从 URDF 构建机器人模型。
        #
        # root_joint=pin.JointModelFreeFlyer()
        # 表示机器人基座是自由浮动的，不是固定在地面上的。
        # 四足机器人走路时，机身会在空间中移动和旋转，所以需要 free-flyer。
        robot = RobotWrapper.BuildFromURDF(
            str(URDF_PATH),
            package_dirs=[str(ROBOT_DIR)],
            root_joint=pin.JointModelFreeFlyer(),
        )

        # robot.model 是机器人结构模型，比如关节、连杆、自由度数量等。
        self.robot = robot
        self.model = robot.model

        # data 是 Pinocchio 用来存放计算结果的地方。
        # 比如每个 link 的位置、速度、质心、Jacobian 等。
        self.data = self.model.createData()

        # q0 是初始姿态，dq0 是初始速度。
        #self.q0 = self.make_standing_q()
        #self.dq0 = np.zeros(self.model.nv)
        self.state = RobotState()
        self.q0 = self.state.get_q()
        self.dq0 = self.state.get_dq()




        # 初始化时先计算一次运动学，让 data 里面有足端位置等结果。
        self.update(self.q0, self.dq0)

    def make_standing_q(self):
        """
        生成一个初始站立姿态 q。

        对 free-flyer 机器人来说，q 的结构是:

        q = [
            base_x, base_y, base_z,              机身位置
            quat_x, quat_y, quat_z, quat_w,      机身四元数姿态
            12 个腿部关节角
        ]

        所以这里总长度是:
        3 + 4 + 12 = 19
        """

        # 机身初始位置，单位是米。
        # z=0.27 表示机身离地大约 0.27m。
        base_pos = np.array([0.0, 0.0, 0.27])

        # 机身初始姿态，用四元数表示。
        # [0, 0, 0, 1] 表示没有旋转。
        base_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0])

        # 12 个关节角，单位是弧度。
        #
        # 每条腿顺序:
        # hip, thigh, calf
        #
        # 这里先用一个比较常见的蹲站姿态。
        joint_angles = np.array([
            0.0, 0.9, -1.8,   # FL 左前腿
            0.0, 0.9, -1.8,   # FR 右前腿
            0.0, 0.9, -1.8,   # RL 左后腿
            0.0, 0.9, -1.8,   # RR 右后腿
        ])

        return np.concatenate([base_pos, base_quat_xyzw, joint_angles])

    def update(self, q, dq):
        """
        根据当前 q 和 dq 更新 Pinocchio 计算结果。

        q  是广义位置:
           包括机身位置、机身姿态、关节角。

        dq 是广义速度:
           包括机身线速度、角速度、关节速度。

        调用这个函数后，self.data 里会更新:
        - 各个 frame 的位置
        - 各个关节的 Jacobian
        - 质心位置等
        """

        # 正向运动学:
        # 根据关节角 q 和速度 dq，计算机器人各个 link/frame 的空间位置。
        pin.forwardKinematics(self.model, self.data, q, dq)

        # 更新所有 frame 的位姿。
        # 如果想读取 foot frame 的位置，这一步很重要。
        pin.updateFramePlacements(self.model, self.data)

        # 计算所有关节的 Jacobian。
        # 后面 get_foot_jacobian 会用到。
        pin.computeJointJacobians(self.model, self.data, q)

        # 计算质心位置和速度。
        pin.centerOfMass(self.model, self.data, q, dq)

    def get_foot_position(self, leg):
        """
        获取某条腿足端在世界坐标系下的位置。

        参数:
        leg: "FL", "FR", "RL", "RR"

        返回:
        一个长度为 3 的 numpy 数组:
        [x, y, z]
        """

        frame_name = FOOT_FRAME_NAMES[leg]
        frame_id = self.model.getFrameId(frame_name)

        # self.data.oMf[frame_id] 是这个 frame 在世界坐标系下的位姿。
        # translation 是其中的位置部分。
        return self.data.oMf[frame_id].translation.copy()

    def get_foot_jacobian(self, leg):
        """
        获取某条腿足端位置对三个腿部关节的 Jacobian。

        Jacobian 可以粗略理解为:
        关节速度如何影响足端速度。

        对单条腿来说，我们只关心:
        hip, thigh, calf 三个关节。

        返回值形状是 (3, 3):
        - 3 行: 足端 x/y/z 三个方向
        - 3 列: hip/thigh/calf 三个关节
        """

        frame_name = FOOT_FRAME_NAMES[leg]
        frame_id = self.model.getFrameId(frame_name)

        # 先获取完整 Jacobian。
        # 它包含整个机器人的所有自由度:
        # base 自由度 + 12 个腿部关节自由度。
        full_jacobian = pin.getFrameJacobian(
            self.model,
            self.data,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        # 前 3 行是线速度部分，也就是足端位置相关的 Jacobian。
        position_jacobian = full_jacobian[0:3, :]

        # 找到这条腿三个关节在模型里的 joint id。
        joint_ids = [self.model.getJointId(name) for name in JOINT_NAMES[leg]]

        # 每个 joint 在速度向量 dq 里对应哪一列。
        velocity_columns = [self.model.joints[joint_id].idx_v for joint_id in joint_ids]

        # 从完整 Jacobian 里只取这条腿三个关节对应的列。
        return position_jacobian[:, velocity_columns]

    def print_summary(self):
        """
        打印当前模型的基本信息，用来检查 URDF 是否加载正确。
        """

        print("URDF:", URDF_PATH)
        print("nq:", self.model.nq)
        print("nv:", self.model.nv)
        print("njoints:", self.model.njoints)
        print()

        print("关节列表:")
        for name in self.model.names:
            print("  ", name)

        print()
        print("足端位置:")
        for leg in LEG_NAMES:
            print(f"  {leg}:", self.get_foot_position(leg))

        print()
        print("足端 3x3 Jacobian 形状:")
        for leg in LEG_NAMES:
            print(f"  {leg}:", self.get_foot_jacobian(leg).shape)


# 当你直接运行这个文件时，会执行下面这几行。
# 如果这个文件被其他脚本 import，则下面不会自动执行。
if __name__ == "__main__":
    model = RealGo2Model()
    model.print_summary()