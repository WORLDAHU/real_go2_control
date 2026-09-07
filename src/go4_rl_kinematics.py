"""URDF-driven kinematics and a simple extend/retract path for the GO4 RL leg."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


RL_JOINT_NAMES = (
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)
RL_FOOT_JOINT = "RL_foot_joint"


def _vector(text: str) -> np.ndarray:
    values = np.asarray([float(value) for value in text.split()], dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError(f"invalid URDF vector: {text!r}")
    return values


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ]
    )


@dataclass(frozen=True)
class JointSpec:
    name: str
    origin_xyz: np.ndarray
    axis: np.ndarray | None
    lower: float | None
    upper: float | None


class Go4RLKinematics:
    """Minimal left-rear-leg model whose geometry comes from GO4.urdf."""

    def __init__(self, joints: tuple[JointSpec, ...], foot: JointSpec):
        if tuple(joint.name for joint in joints) != RL_JOINT_NAMES:
            raise ValueError("unexpected GO4 RL joint chain")
        self.joints = joints
        self.foot = foot
        self.lower = np.asarray([joint.lower for joint in joints], dtype=float)
        self.upper = np.asarray([joint.upper for joint in joints], dtype=float)

    @classmethod
    def from_urdf(cls, path: str | Path) -> "Go4RLKinematics":
        root = ET.parse(Path(path)).getroot()
        by_name = {joint.get("name"): joint for joint in root.findall("joint")}

        def parse(name: str, *, fixed: bool = False) -> JointSpec:
            element = by_name.get(name)
            if element is None:
                raise ValueError(f"URDF is missing {name}")
            origin = element.find("origin")
            xyz = _vector(origin.get("xyz", "0 0 0")) if origin is not None else np.zeros(3)
            if fixed:
                return JointSpec(name, xyz, None, None, None)
            axis_element = element.find("axis")
            limit = element.find("limit")
            if axis_element is None or limit is None:
                raise ValueError(f"URDF joint {name} needs axis and limits")
            return JointSpec(
                name,
                xyz,
                _vector(axis_element.get("xyz")),
                float(limit.get("lower")),
                float(limit.get("upper")),
            )

        return cls(
            tuple(parse(name) for name in RL_JOINT_NAMES),
            parse(RL_FOOT_JOINT, fixed=True),
        )

    def validate_q(self, q_rad) -> np.ndarray:
        q = np.asarray(q_rad, dtype=float)
        if q.shape != (3,) or not np.all(np.isfinite(q)):
            raise ValueError("q_rad must contain three finite joint angles")
        if np.any(q < self.lower - 1e-10) or np.any(q > self.upper + 1e-10):
            raise ValueError(
                f"joint target outside URDF limits: q={np.degrees(q)}, "
                f"lower={np.degrees(self.lower)}, upper={np.degrees(self.upper)}"
            )
        return q

    def foot_position(self, q_rad) -> np.ndarray:
        """Return RL foot origin in the base_link frame."""

        q = self.validate_q(q_rad)
        rotation = np.eye(3)
        position = np.zeros(3)
        for joint, angle in zip(self.joints, q):
            position = position + rotation @ joint.origin_xyz
            rotation = rotation @ _rotation(joint.axis, angle)
        return position + rotation @ self.foot.origin_xyz

    def retracted_q(self) -> np.ndarray:
        """URDF retracted reference: neutral hip, thigh upper, calf lower."""

        q = np.array([0.0, self.upper[1], self.lower[2]], dtype=float)
        return self.validate_q(q)

    def q_at_extension(self, depth_m: float, q_seed=None) -> np.ndarray:
        """Solve the vertical foot pose ``depth_m`` below the retracted pose."""

        if not math.isfinite(depth_m) or depth_m < 0.0:
            raise ValueError("depth_m must be non-negative and finite")
        q_retracted = self.retracted_q()
        seed = q_retracted if q_seed is None else self.validate_q(q_seed)
        target = self.foot_position(q_retracted) + np.array([0.0, 0.0, -depth_m])
        return self.solve_foot_target(target, seed, fixed_hip=q_retracted[0])

    def extension_keyframes(self, depths_m) -> np.ndarray:
        """Solve an ordered sequence of vertical extension depths continuously."""

        depths = np.asarray(depths_m, dtype=float)
        if depths.ndim != 1 or len(depths) < 1 or not np.all(np.isfinite(depths)):
            raise ValueError("depths_m must be a non-empty finite sequence")
        if np.any(depths < 0.0):
            raise ValueError("extension depths must be non-negative")
        q_seed = self.retracted_q()
        solved = []
        for depth in depths:
            q_seed = self.q_at_extension(float(depth), q_seed)
            solved.append(q_seed.copy())
        return np.asarray(solved)

    def solve_foot_target(
        self,
        target_xyz,
        q_seed,
        *,
        fixed_hip: float = 0.0,
        tolerance_m: float = 2e-6,
        max_iterations: int = 100,
    ) -> np.ndarray:
        """Solve thigh/calf angles while keeping the hip angle fixed."""

        target = np.asarray(target_xyz, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("target_xyz must contain three finite coordinates")
        q = np.clip(np.asarray(q_seed, dtype=float), self.lower, self.upper)
        q[0] = float(fixed_hip)
        if not self.lower[0] <= q[0] <= self.upper[0]:
            raise ValueError("fixed hip target is outside its URDF limits")

        damping = 1e-7
        epsilon = 1e-6
        for _ in range(max_iterations):
            position = self.foot_position(q)
            error = target - position
            if np.linalg.norm(error) <= tolerance_m:
                return q
            jacobian = np.zeros((3, 2))
            for column, joint_index in enumerate((1, 2)):
                shifted = q.copy()
                shifted[joint_index] = min(q[joint_index] + epsilon, self.upper[joint_index])
                actual_step = shifted[joint_index] - q[joint_index]
                if actual_step <= 0.0:
                    shifted[joint_index] = max(q[joint_index] - epsilon, self.lower[joint_index])
                    actual_step = shifted[joint_index] - q[joint_index]
                jacobian[:, column] = (
                    self.foot_position(shifted) - position
                ) / actual_step
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3), error
            )
            delta = np.clip(delta, -0.08, 0.08)
            q[1:] = np.clip(q[1:] + delta, self.lower[1:], self.upper[1:])

        residual = np.linalg.norm(target - self.foot_position(q))
        raise ValueError(f"foot target is unreachable; residual={residual:.6f} m")

    def extension_cycle(self, stroke_m: float = 0.06, samples_per_leg: int = 100):
        """Generate retracted -> vertically down -> retracted joint targets."""

        if not math.isfinite(stroke_m) or stroke_m <= 0.0:
            raise ValueError("stroke_m must be positive and finite")
        if samples_per_leg < 2:
            raise ValueError("samples_per_leg must be at least 2")

        q_retracted = self.retracted_q()
        foot_retracted = self.foot_position(q_retracted)
        q_seed = q_retracted.copy()
        outward = []
        for index in range(samples_per_leg + 1):
            ratio = index / samples_per_leg
            blend = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
            target = foot_retracted + np.array([0.0, 0.0, -stroke_m * blend])
            q_seed = self.solve_foot_target(target, q_seed, fixed_hip=q_retracted[0])
            outward.append(q_seed.copy())
        return np.asarray(outward + outward[-2::-1])
