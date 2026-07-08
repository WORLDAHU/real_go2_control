import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

# ============================================================
# 路径设置
# ============================================================

# 当前脚本位置:
# real_go2_control/scripts/18_view_jump_with_squat_push.py
#
# parents[1] 表示向上两级，得到项目根目录:
# real_go2_control/
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# 让 Python 能找到 src/ 里的 RealGo2Model
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model


# ============================================================
# MuJoCo 模型路径
# ============================================================

# 这个 XML 是前面生成的:
# - freejoint 自由基座
# - 12 个 motor actuator
# - 8 N·m 力矩限制
# - 简化机身显示 + 腿部 STL 显示
MJCF_PATH = PROJECT_ROOT / "models" / "mujoco" / "go2_freebase_actuated.xml"


# ============================================================
# 12 个电机关节顺序
# ============================================================

# 这个顺序非常重要。
# 后面 q、qd、ctrl 都按这个顺序组成 12 维数组。
JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


# ============================================================
# 控制参数
# ============================================================

# 每个电机最大输出力矩，单位 N·m。
# 当前学习目标统一限制成 8 N·m。
TORQUE_LIMIT = 8.0

# 动作分段时间，单位秒。
#
# prepare:
#   从站立姿态慢慢蹲下。
#
# push:
#   从蹲姿快速伸腿，用来蹬地。
#
# flight:
#   空中阶段，腿保持伸展，但控制不要太硬。
#
# landing:
#   落地阶段，这里做成两段缓冲:
#   1. 先进入软着陆小蹲姿
#   2. 再慢慢恢复站立
PREPARE_TIME = 1.0
PUSH_TIME = 0.25
FLIGHT_TIME = 0.5
LANDING_TIME = 3



# 落地阶段中，有多少比例用于“吸收冲击”。
# 0.45 表示 landing 时间的前 45% 用来从伸腿姿态过渡到软着陆姿态。
LANDING_ABSORB_RATIO = 0.6

# 两次重复播放之间的等待时间，单位秒。
REPLAY_PAUSE_TIME = 15

# 一次完整动作的总时间。
TOTAL_TIME = PREPARE_TIME + PUSH_TIME + FLIGHT_TIME + LANDING_TIME


# ============================================================
# PD 增益
# ============================================================

# push 阶段:
# 希望腿比较有力地伸出去，所以 thigh/calf 的 kp 稍大。
KP_PUSH = np.array([16.0, 42.0, 42.0] * 4)
KD_PUSH = np.array([0.8, 1.6, 1.6] * 4)

# landing_absorb 阶段:
# kp 稍低，kd 稍高。
# 直觉上类似“软弹簧 + 强阻尼”，用来吸收落地冲击。
KP_LAND = np.array([14.0, 24.0, 24.0] * 4)
KD_LAND = np.array([1.4, 3.2, 3.2] * 4)

# landing_recover / done 阶段:
# 从软着陆姿态恢复站立，用中等刚度。
KP_RECOVER = np.array([18.0, 30.0, 30.0] * 4)
KD_RECOVER = np.array([1.0, 2.4, 2.4] * 4)


def smoothstep(s):
    """
    平滑插值函数。

    输入:
        s 从 0 到 1

    输出:
        也是 0 到 1

    特点:
        起点和终点速度都是 0，动作看起来更顺滑。
    """

    s = np.clip(s, 0.0, 1.0)
    return 3.0 * s * s - 2.0 * s * s * s


def make_pose(hip, thigh, calf):
    """
    生成四条腿相同姿态的 12 维关节角数组。

    每条腿顺序:
        hip, thigh, calf

    输入:
        hip, thigh, calf 是单条腿的 3 个关节角。

    输出:
        [FL三关节, FR三关节, RL三关节, RR三关节]
    """

    return np.array([hip, thigh, calf] * 4, dtype=float)


def get_joint_q_qd(model, data):
    """
    从 MuJoCo 当前状态中读取 12 个关节角 q 和关节速度 qd。

    返回:
        q:
            12 个关节角

        qd:
            12 个关节速度
    """

    q = np.zeros(12)
    qd = np.zeros(12)

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

        qpos_id = model.jnt_qposadr[joint_id]
        dof_id = model.jnt_dofadr[joint_id]

        q[i] = data.qpos[qpos_id]
        qd[i] = data.qvel[dof_id]

    return q, qd


def set_initial_pose(model, data, q_init):
    """
    重置 MuJoCo 仿真，并设置初始姿态。

    这里设置:
    - base 位置 z = 0.25
    - base 姿态为单位四元数
    - 12 个腿部关节角为 q_init
    """

    mujoco.mj_resetData(model, data)

    # freejoint 的 qpos 顺序:
    # x, y, z, qw, qx, qy, qz
    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    for i, joint_name in enumerate(JOINT_ORDER):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_id = model.jnt_qposadr[joint_id]
        data.qpos[qpos_id] = q_init[i]

    mujoco.mj_forward(model, data)


def get_target_pose(t, q_stand, q_crouch, q_extend, q_soft_land):
    """
    根据当前动作时间 t，返回目标关节姿态 q_des、PD 增益、阶段名。

    动作阶段:
    1. prepare:
       站立 -> 蹲下

    2. push:
       蹲下 -> 快速伸腿

    3. flight:
       空中保持伸腿，但控制刚度降低

    4. landing_absorb:
       伸腿 -> 软着陆姿态

    5. landing_recover:
       软着陆姿态 -> 站立姿态
    """

    if t < PREPARE_TIME:
        # 站立 -> 蹲下
        a = smoothstep(t / PREPARE_TIME)
        q_des = (1.0 - a) * q_stand + a * q_crouch
        return q_des, KP_LAND, KD_LAND, "prepare"

    if t < PREPARE_TIME + PUSH_TIME:
        # 蹲下 -> 快速伸腿
        s = (t - PREPARE_TIME) / PUSH_TIME
        a = smoothstep(s)
        q_des = (1.0 - a) * q_crouch + a * q_extend
        return q_des, KP_PUSH, KD_PUSH, "push"

    if t < PREPARE_TIME + PUSH_TIME + FLIGHT_TIME:
        # 空中保持伸腿，但不要太硬。
        # 如果这里太硬，落地时容易弹飞或抖动。
        return q_extend, 0.45 * KP_LAND, KD_LAND, "flight"

    if t < TOTAL_TIME:
        # 两段式落地缓冲。
        landing_t = t - PREPARE_TIME - PUSH_TIME - FLIGHT_TIME
        absorb_time = LANDING_TIME * LANDING_ABSORB_RATIO

        if landing_t < absorb_time:
            # 第一段:
            # 伸腿姿态 -> 软着陆姿态
            s = landing_t / absorb_time
            a = smoothstep(s)
            q_des = (1.0 - a) * q_extend + a * q_soft_land
            return q_des, KP_LAND, KD_LAND, "landing_absorb"

        # 第二段:
        # 软着陆姿态 -> 站立姿态
        s = (landing_t - absorb_time) / (LANDING_TIME - absorb_time)
        a = smoothstep(s)
        q_des = (1.0 - a) * q_soft_land + a * q_stand
        return q_des, KP_RECOVER, KD_RECOVER, "landing_recover"

    # 动作结束后保持站立。
    return q_stand, KP_RECOVER, KD_RECOVER, "done"


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)

    robot_model = RealGo2Model()

    # 站立姿态来自 RobotState / RealGo2Model 里的初始 q0。
    q_stand = robot_model.q0[7:19].copy()

    # 蹲姿:
    # thigh 更大、calf 更负，表示腿更弯。
    q_crouch = make_pose(0.0, 1.25, -2.35)

    # 伸腿姿态:
    # thigh 更小、calf 不那么负，表示腿更伸直。
    q_extend = make_pose(0.0, 0.35, -0.95)

    # 软着陆姿态:
    # 比站立稍微蹲一点，用来接住落地冲击。
    q_soft_land = make_pose(0.0, 1.05, -2.05)

    set_initial_pose(model, data, q_stand)

    print("viewer started")
    print("free camera enabled")
    print("torque limit:", TORQUE_LIMIT)
    print("q_stand:", q_stand[:3])
    print("q_crouch:", q_crouch[:3])
    print("q_extend:", q_extend[:3])
    print("q_soft_land:", q_soft_land[:3])
    print("replay pause:", REPLAY_PAUSE_TIME)
    print("close window to stop")
    print()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 初始视角。
        # 这里没有每帧跟随机器人，所以你可以自由拖动视角。
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [0.0, 0.0, 0.25]

        replay_start = time.time()

        while viewer.is_running():
            elapsed = time.time() - replay_start

            # 一次动作结束后，等待 REPLAY_PAUSE_TIME 秒，然后重播。
            if elapsed > TOTAL_TIME + REPLAY_PAUSE_TIME:
                set_initial_pose(model, data, q_stand)
                replay_start = time.time()
                print("replay jump")
                viewer.sync()
                time.sleep(0.3)
                continue

            # MuJoCo 当前仿真时间。
            # 重置仿真时 data.time 会回到 0。
            t = data.time

            # 获取当前阶段的目标关节角和 PD 增益。
            q_des, kp, kd, phase = get_target_pose(
                t,
                q_stand,
                q_crouch,
                q_extend,
                q_soft_land,
            )

            # 读取当前关节角和关节速度。
            q, qd = get_joint_q_qd(model, data)

            # 关节空间 PD 控制:
            #
            # ctrl = kp * 位置误差 - kd * 速度
            #
            # 这里 ctrl 会直接发给 MuJoCo 的 12 个 motor actuator。
            ctrl = kp * (q_des - q) - kd * qd

            # 力矩限幅，模拟每个电机最大 8 N·m。
            ctrl = np.clip(ctrl, -TORQUE_LIMIT, TORQUE_LIMIT)

            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

            # 注意:
            # 这里没有写 viewer.cam.lookat = data.qpos[0:3]
            # 所以视角不会自动跟随机器人，可以用鼠标自由调整。
            viewer.sync()

            # 让播放速度接近真实时间。
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
