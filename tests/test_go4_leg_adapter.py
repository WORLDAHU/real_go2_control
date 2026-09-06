import math
import sys
import unittest
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_leg_adapter import (
    KneeTransmission,
    joint_delta_to_motor_output_delta_deg,
    motor_output_delta_to_joint_delta_deg,
)
from unitree_daisy_chain import unwrap_near


class Go4LegAdapterTests(unittest.TestCase):
    def test_knee_gear_ratio_16_to_28(self):
        transmission = KneeTransmission()
        self.assertTrue(
            math.isclose(transmission.motor_to_knee_ratio, 16.0 / 28.0)
        )
        self.assertTrue(
            math.isclose(transmission.motor_output_to_knee_deg(1.75), 1.0)
        )
        self.assertTrue(
            math.isclose(transmission.knee_to_motor_output_deg(1.0), 1.75)
        )

    def test_knee_round_trip_with_direction_and_zero(self):
        transmission = KneeTransmission(direction=-1.0, knee_zero_deg=-70.0)
        for knee_deg in (-150.0, -100.0, -70.0, -64.5):
            motor_deg = transmission.knee_to_motor_output_deg(knee_deg)
            recovered = transmission.motor_output_to_knee_deg(motor_deg)
            self.assertTrue(math.isclose(recovered, knee_deg, abs_tol=1e-12))

    def test_unwrap_near_selects_nearest_single_turn_branch(self):
        self.assertTrue(
            math.isclose(
                unwrap_near(0.1, 2.0 * math.pi + 0.2),
                2.0 * math.pi + 0.1,
            )
        )

    def test_relative_direct_joint_mapping(self):
        for role in ("hip", "thigh"):
            self.assertEqual(
                joint_delta_to_motor_output_delta_deg(role, 2.0, -1.0),
                -2.0,
            )
            self.assertEqual(
                motor_output_delta_to_joint_delta_deg(role, -2.0, -1.0),
                2.0,
            )

    def test_relative_knee_mapping_round_trip(self):
        for direction in (-1.0, 1.0):
            motor_delta = joint_delta_to_motor_output_delta_deg(
                "knee", 1.0, direction
            )
            self.assertTrue(math.isclose(abs(motor_delta), 1.75))
            recovered = motor_output_delta_to_joint_delta_deg(
                "knee", motor_delta, direction
            )
            self.assertTrue(math.isclose(recovered, 1.0))


if __name__ == "__main__":
    unittest.main()
