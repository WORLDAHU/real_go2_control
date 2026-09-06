"""GO4 left-rear leg naming and external transmission conversions."""

from __future__ import annotations

from dataclasses import dataclass
import math


# Mechanical order from the body towards the foot.  IDs are initial defaults;
# confirm them with scripts/41_scan_daisy_chain.py on the assembled bus.
RL_MOTOR_ORDER = ("hip", "thigh", "knee")
DEFAULT_RL_MOTOR_IDS = {"hip": 0, "thigh": 1, "knee": 2}


@dataclass(frozen=True)
class KneeTransmission:
    """16-tooth motor pinion driving a 28-tooth knee gear.

    The parallelogram is treated as 1:1.  ``direction`` and ``knee_zero_deg``
    remain calibration parameters because neither can be proven from the URDF.
    """

    motor_teeth: int = 16
    knee_teeth: int = 28
    direction: float = 1.0
    knee_zero_deg: float = 0.0

    def __post_init__(self):
        if self.motor_teeth <= 0 or self.knee_teeth <= 0:
            raise ValueError("gear tooth counts must be positive")
        if self.direction not in (-1.0, 1.0):
            raise ValueError("direction must be +1 or -1")
        if not math.isfinite(self.knee_zero_deg):
            raise ValueError("knee_zero_deg must be finite")

    @property
    def motor_to_knee_ratio(self) -> float:
        return self.motor_teeth / self.knee_teeth

    def motor_output_to_knee_deg(self, motor_output_deg: float) -> float:
        return (
            self.knee_zero_deg
            + self.direction * float(motor_output_deg) * self.motor_to_knee_ratio
        )

    def knee_to_motor_output_deg(self, knee_deg: float) -> float:
        return (
            (float(knee_deg) - self.knee_zero_deg)
            / self.direction
            / self.motor_to_knee_ratio
        )
