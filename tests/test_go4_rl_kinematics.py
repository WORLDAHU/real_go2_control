import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from go4_rl_kinematics import Go4RLKinematics


URDF = PROJECT_ROOT / "models" / "go4" / "GO4.urdf"


class Go4RLKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.model = Go4RLKinematics.from_urdf(URDF)

    def test_parameters_are_loaded_from_go4_urdf(self):
        np.testing.assert_allclose(
            np.degrees(self.model.lower), [-48.0, -200.0, -159.5000025], atol=1e-5
        )
        np.testing.assert_allclose(
            np.degrees(self.model.upper), [48.0, 89.9999985, -64.4999995], atol=1e-5
        )
        np.testing.assert_allclose(self.model.foot.origin_xyz, [0.0, 0.0, -0.194])

    def test_retracted_pose_uses_urdf_limits(self):
        q = self.model.retracted_q()
        self.assertEqual(q[0], 0.0)
        self.assertEqual(q[1], self.model.upper[1])
        self.assertEqual(q[2], self.model.lower[2])

    def test_extension_cycle_moves_only_down_and_returns(self):
        trajectory = self.model.extension_cycle(stroke_m=0.04, samples_per_leg=30)
        foot = np.asarray([self.model.foot_position(q) for q in trajectory])
        middle = len(trajectory) // 2
        np.testing.assert_allclose(trajectory[0], trajectory[-1], atol=1e-9)
        np.testing.assert_allclose(foot[0, :2], foot[middle, :2], atol=2e-6)
        self.assertAlmostEqual(foot[middle, 2] - foot[0, 2], -0.04, places=5)
        self.assertTrue(np.all(trajectory >= self.model.lower - 1e-10))
        self.assertTrue(np.all(trajectory <= self.model.upper + 1e-10))


if __name__ == "__main__":
    unittest.main()
