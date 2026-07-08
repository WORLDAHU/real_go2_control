import mujoco
import numpy as np


MOTOR_JOINTS = [
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]

# MuJoCo 会把 RL_foot fixed joint 合并到 RL_calf。
# 所以真正的足端接触点不是 RL_foot body，而是 RL_calf 上的足端球 geom。
FOOT_CONTACT_GEOM = "RL_foot_contact_geom"


class SingleLegSliderController:
    """
    单腿导轨跳跃控制器。

    控制逻辑:
    1. 以 motor0/RL_hip 为参考点，规划它正下方的足端竖直轨迹。
    2. 用 IK 把足端目标位置转换成 3 个关节角。
    3. 用关节 PD 输出 3 个电机力矩。

    这个类不负责生成 URDF/MJCF。
    """

    def __init__(self, model):
        self.model = model
        self.ik_data = mujoco.MjData(model)

        # 动作时间参数，单位秒。
        self.prepare_time = 0.8
        self.push_time = 0.45
        self.flight_time = 0.50
        self.landing_time = 3.0
        self.landing_absorb_ratio = 0.70
        self.update_total_time()

        # 足端目标相对 motor0 正下方的水平偏移。
        # [0, 0] 表示足端目标就在 motor0 正下方。
        self.target_foot_xy = np.array([0.0, 0.0790492])

        # 足端 z 方向轨迹参数。
        # 正数: 脚相对 motor0 更近，腿更蹲。
        # 负数: 脚相对 motor0 更远，腿更伸。
        self.crouch_dz = 0.06
        self.extend_dz = -0.10
        self.soft_land_dz = 0.04

        # 分阶段力矩限幅。
        self.phase_torque_limits = {
            "prepare": 3.0,
            "push": 8.0,
            "flight": 1.2,
            "landing_absorb": 3.5,
            "landing_recover": 2.5,
            "done": 2.0,
        }

        self.kp_prepare = np.array([8.0, 18.0, 18.0])
        self.kd_prepare = np.array([0.8, 2.0, 2.0])

        self.kp_push = np.array([10.0, 34.0, 34.0])
        self.kd_push = np.array([1.0, 2.2, 2.2])

        self.kp_land = np.array([6.0, 12.0, 12.0])
        self.kd_land = np.array([2.0, 5.0, 5.0])

        self.kp_recover = np.array([8.0, 16.0, 16.0])
        self.kd_recover = np.array([1.5, 3.5, 3.5])

        # IK 初始猜测。
        self.q_stand_seed = np.array([0.0, 0.9, -1.8])
        self.q_seed = self.q_stand_seed.copy()

        self.is_initialized = False

    def update_total_time(self):
        self.total_time = (
            self.prepare_time
            + self.push_time
            + self.flight_time
            + self.landing_time
        )

    def initialize_targets(self, data):
        """
        根据当前模型建立足端轨迹关键点。

        所有足端点都在 slider_link 坐标系下。
        xy: motor0 正下方
        z: 取当前站立姿态的足端高度，再加 dz
        """

        self.update_total_time()
        self.set_initial_pose(data, self.q_stand_seed)

        p_stand_measured = self.foot_pos_in_slider_frame(data)
        motor0_pos = self.motor0_pos_in_slider_frame(data)

        target_foot_xy = motor0_pos[:2] + self.target_foot_xy

        self.p_stand = np.array([
            target_foot_xy[0],
            target_foot_xy[1],
            p_stand_measured[2],
        ])
        self.p_crouch = self.p_stand + np.array([0.0, 0.0, self.crouch_dz])
        self.p_extend = self.p_stand + np.array([0.0, 0.0, self.extend_dz])
        self.p_soft_land = self.p_stand + np.array([0.0, 0.0, self.soft_land_dz])

        self.q_stand = self.solve_ik_foot_target(self.p_stand, self.q_stand_seed)
        self.q_crouch = self.solve_ik_foot_target(self.p_crouch, self.q_stand)
        self.q_extend = self.solve_ik_foot_target(self.p_extend, self.q_crouch)
        self.q_soft_land = self.solve_ik_foot_target(self.p_soft_land, self.q_stand)

        self.q_seed = self.q_stand.copy()
        self.set_initial_pose(data, self.q_stand)
        self.is_initialized = True

    def set_initial_pose(self, data, q_init):
        mujoco.mj_resetData(self.model, data)

        slider_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "slider_z_joint"
        )
        data.qpos[self.model.jnt_qposadr[slider_id]] = 0.0

        for i, joint_name in enumerate(MOTOR_JOINTS):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            data.qpos[self.model.jnt_qposadr[joint_id]] = q_init[i]

        mujoco.mj_forward(self.model, data)

    def reset_for_replay(self, data):
        self.q_seed = self.q_stand.copy()
        self.set_initial_pose(data, self.q_stand)

    def compute_control(self, data):
        """
        每个仿真步调用一次。

        返回:
            ctrl: 3 个电机力矩
            info: 当前阶段、目标点、目标关节角等调试信息
        """

        if not self.is_initialized:
            self.initialize_targets(data)

        p_des, kp, kd, phase = self.foot_target_at_time(data.time)

        q_des = self.solve_ik_foot_target(p_des, self.q_seed)
        self.q_seed = q_des.copy()

        q, qd = self.get_motor_q_qd(data)

        ctrl = kp * (q_des - q) - kd * qd
        limit = self.phase_torque_limits[phase]
        ctrl = np.clip(ctrl, -limit, limit)

        info = {
            "phase": phase,
            "p_des": p_des,
            "q_des": q_des,
            "q": q,
            "qd": qd,
            "limit": limit,
        }

        return ctrl, info

    def foot_target_at_time(self, t):
        if t < self.prepare_time:
            a = self.min_jerk(t / self.prepare_time)
            return (
                (1.0 - a) * self.p_stand + a * self.p_crouch,
                self.kp_prepare,
                self.kd_prepare,
                "prepare",
            )

        if t < self.prepare_time + self.push_time:
            s = (t - self.prepare_time) / self.push_time
            a = self.min_jerk(s)
            p_des = (1.0 - a) * self.p_crouch + a * self.p_extend

            # push 末尾降低刚度，减少阶段切换时的力矩尖峰。
            taper = 1.0 - 0.35 * self.smoothstep((s - 0.75) / 0.25)
            return p_des, taper * self.kp_push, self.kd_push, "push"

        if t < self.prepare_time + self.push_time + self.flight_time:
            return self.p_extend, 0.45 * self.kp_land, self.kd_land, "flight"

        if t < self.total_time:
            landing_t = t - self.prepare_time - self.push_time - self.flight_time
            absorb_time = self.landing_time * self.landing_absorb_ratio

            if landing_t < absorb_time:
                a = self.min_jerk(landing_t / absorb_time)
                return (
                    (1.0 - a) * self.p_extend + a * self.p_soft_land,
                    self.kp_land,
                    self.kd_land,
                    "landing_absorb",
                )

            a = self.min_jerk(
                (landing_t - absorb_time) / (self.landing_time - absorb_time)
            )
            return (
                (1.0 - a) * self.p_soft_land + a * self.p_stand,
                self.kp_recover,
                self.kd_recover,
                "landing_recover",
            )

        return self.p_stand, self.kp_recover, self.kd_recover, "done"

    def get_motor_q_qd(self, data):
        q = np.zeros(3)
        qd = np.zeros(3)

        for i, joint_name in enumerate(MOTOR_JOINTS):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            q[i] = data.qpos[self.model.jnt_qposadr[joint_id]]
            qd[i] = data.qvel[self.model.jnt_dofadr[joint_id]]

        return q, qd

    def motor0_pos_in_slider_frame(self, data):
        motor0_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "RL_hip"
        )
        slider_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "slider_link"
        )

        return data.xpos[motor0_id].copy() - data.xpos[slider_id].copy()

    def foot_pos_in_slider_frame(self, data):
        foot_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, FOOT_CONTACT_GEOM
        )
        slider_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "slider_link"
        )

        if foot_geom_id < 0:
            raise RuntimeError(f"找不到足端接触 geom: {FOOT_CONTACT_GEOM}")

        return data.geom_xpos[foot_geom_id].copy() - data.xpos[slider_id].copy()

    def set_motor_q_for_ik(self, q):
        mujoco.mj_resetData(self.model, self.ik_data)

        slider_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "slider_z_joint"
        )
        self.ik_data.qpos[self.model.jnt_qposadr[slider_id]] = 0.0

        for i, joint_name in enumerate(MOTOR_JOINTS):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            self.ik_data.qpos[self.model.jnt_qposadr[joint_id]] = q[i]

        mujoco.mj_forward(self.model, self.ik_data)

    def solve_ik_foot_target(self, target_pos_slider, q_seed):
        q = q_seed.copy()
        eps = 1e-4
        damping = 1e-4

        for _ in range(40):
            self.set_motor_q_for_ik(q)
            pos = self.foot_pos_in_slider_frame(self.ik_data)
            err = target_pos_slider - pos

            if np.linalg.norm(err) < 1e-4:
                break

            J = np.zeros((3, 3))

            for j in range(3):
                q2 = q.copy()
                q2[j] += eps
                self.set_motor_q_for_ik(q2)
                pos2 = self.foot_pos_in_slider_frame(self.ik_data)
                J[:, j] = (pos2 - pos) / eps

            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), err)
            dq = np.clip(dq, -0.12, 0.12)
            q = self.clip_to_joint_limits(q + dq)

        return q

    def clip_to_joint_limits(self, q):
        out = q.copy()

        for i, joint_name in enumerate(MOTOR_JOINTS):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )

            if self.model.jnt_limited[joint_id]:
                lo, hi = self.model.jnt_range[joint_id]
                out[i] = np.clip(out[i], lo, hi)

        return out

    @staticmethod
    def smoothstep(s):
        s = np.clip(s, 0.0, 1.0)
        return 3.0 * s * s - 2.0 * s * s * s

    @staticmethod
    def min_jerk(s):
        s = np.clip(s, 0.0, 1.0)
        return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
