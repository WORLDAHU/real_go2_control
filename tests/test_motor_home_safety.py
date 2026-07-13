import importlib.util
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))


def load_script(name, filename):
    spec = importlib.util.spec_from_file_location(name, PROJECT_ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


calibrate = load_script("calibrate_motor_home", "33_calibrate_motor_home.py")
bridge = load_script("real_unitree_leg_bridge", "32_real_unitree_leg_bridge.py")
single_motor = load_script("test_leg_motor_angle", "37_test_leg_motor_angle.py")


class FakeMotorData:
    def __init__(self):
        self.correct = False
        self.motor_id = 0
        self.merror = 0
        self.q = 0.0


class FakeSerial:
    def __init__(self, replies):
        self.replies = iter(replies)

    def sendRecv(self, cmd, data):
        ok, correct, motor_id, merror, q = next(self.replies)
        data.correct = correct
        data.motor_id = motor_id
        data.merror = merror
        data.q = q
        return ok


class FakeSDK:
    class MotorType:
        GO_M8010_6 = object()

    class MotorMode:
        FOC = object()

    MotorData = FakeMotorData

    def __init__(self, replies):
        self.replies = replies

    @staticmethod
    def MotorCmd():
        return type("FakeMotorCmd", (), {})()

    def SerialPort(self, _port):
        return FakeSerial(self.replies)

    @staticmethod
    def queryMotorMode(_motor_type, _motor_mode):
        return 1


class MotorHomeSafetyTests(unittest.TestCase):
    def test_calibration_rejects_timeouts_and_garbage_q(self):
        replies = [(False, False, 2, 0, 7.3e24)] * 5
        with self.assertRaisesRegex(RuntimeError, "valid replies 0/5"):
            calibrate.read_current_rotor(FakeSDK(replies), "/dev/null", 2, 5, 0.0)

    def test_calibration_accepts_stable_valid_replies(self):
        replies = [(True, True, 2, 0, 1.0 + i * 0.001) for i in range(5)]
        q_home, _data, valid, spread = calibrate.read_current_rotor(
            FakeSDK(replies), "/dev/null", 2, 5, 0.0
        )
        self.assertEqual(valid, 5)
        self.assertAlmostEqual(q_home, 1.002)
        self.assertAlmostEqual(spread, 0.004)

    def test_bridge_rejects_saved_garbage_home(self):
        entry = {"q_home": 7.3e24, "gear": 6.33}
        with self.assertRaisesRegex(ValueError, "invalid q_home"):
            bridge.validate_home_numeric(entry, "hip_motor", 6.33)

    def test_single_motor_rejects_saved_garbage_home(self):
        entry = {"q_home": 7.3e24, "gear": 6.33}
        with self.assertRaisesRegex(ValueError, "invalid q_home"):
            single_motor.validate_home_numeric(entry, "hip_motor")

    def test_bridge_rejects_out_of_range_instead_of_clamping(self):
        with self.assertRaisesRegex(ValueError, "outside safe range"):
            bridge.clamp_finite(-130.0, -120.0, 0.0, "thigh_motor")

    def test_bridge_applies_conservative_speed_limit(self):
        controller = bridge.RealUnitreeLegBridge(
            motors_cfg=bridge.DEFAULT_MOTORS,
            enable_motors=False,
        )
        controller.motors_ready = True
        accepted = controller.set_targets(
            {"hip_motor": 0.0, "thigh_motor": -20.0, "calf_motor": -40.0},
            ramp_time=0.05,
        )
        self.assertEqual(accepted["thigh_motor"], -20.0)
        self.assertEqual(controller.last_accepted_ramp_time, 2.0)


if __name__ == "__main__":
    unittest.main()
