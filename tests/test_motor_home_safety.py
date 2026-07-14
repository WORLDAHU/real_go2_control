import importlib.util
import sys
import time
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
release_motor = load_script("release_leg_motors", "39_release_leg_motors.py")


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
    def test_calf_home_angle_is_derived_from_fourbar_geometry(self):
        adapter = bridge.RealLegCommandAdapter()
        self.assertAlmostEqual(
            adapter.fourbar.knee_pitch_home_deg,
            adapter.calf_motor_to_knee_pitch(0.0),
            places=12,
        )
        self.assertAlmostEqual(
            bridge.EXPECTED_CALIBRATION_METADATA["calf_motor"]["common_deg"],
            adapter.fourbar.knee_pitch_home_deg,
            places=12,
        )

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

    def test_home_reference_is_aligned_across_one_rotor_turn(self):
        stored_home = -10.593168
        true_joint_deg = 0.25
        raw_now = (
            stored_home
            + 2.0 * bridge.math.pi
            + bridge.math.radians(true_joint_deg) * 6.33
        )
        aligned_home = bridge.unwrap_near(stored_home, raw_now)
        recovered = bridge.rotor_to_joint(raw_now, aligned_home, 6.33)
        self.assertAlmostEqual(recovered, true_joint_deg, places=9)

    def test_calibration_unwraps_samples_across_plus_minus_pi(self):
        replies = [
            (True, True, 2, 0, 3.13),
            (True, True, 2, 0, -3.14),
            (True, True, 2, 0, -3.13),
            (True, True, 2, 0, -3.12),
            (True, True, 2, 0, -3.11),
        ]
        q_home, _data, valid, spread = calibrate.read_current_rotor(
            FakeSDK(replies), "/dev/null", 2, 5, 0.0
        )
        self.assertEqual(valid, 5)
        self.assertLess(spread, 0.05)
        self.assertGreater(q_home, 3.13)

    def test_bridge_runtime_rejects_timeout_bad_frame_wrong_id_and_fault(self):
        runtime = object.__new__(bridge.MotorRuntime)
        runtime.cfg = {"label": "hip", "id": 2}
        runtime.data = FakeMotorData()
        runtime.position_valid = False
        cases = (
            (False, True, 2, 0, "timeout"),
            (True, False, 2, 0, "CRC"),
            (True, True, 1, 0, "reply id"),
            (True, True, 2, 5, "motor fault"),
        )
        for ok, correct, motor_id, merror, message in cases:
            runtime.data.correct = correct
            runtime.data.motor_id = motor_id
            runtime.data.merror = merror
            runtime.data.q = 1.0
            with self.subTest(message=message):
                with self.assertRaisesRegex(RuntimeError, message):
                    runtime.validate_reply(ok)

    def test_single_motor_command_rejects_motor_fault(self):
        sdk = FakeSDK([(True, True, 2, 5, 1.0)])
        serial = sdk.SerialPort("/dev/null")
        with self.assertRaisesRegex(RuntimeError, "motor fault"):
            single_motor.send_cmd(
                sdk, serial, sdk.MotorCmd(), sdk.MotorData(), 2,
                q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0,
            )

    def test_release_command_rejects_timeout_before_using_position(self):
        sdk = FakeSDK([(False, False, 2, 0, 7.3e24)])
        serial = sdk.SerialPort("/dev/null")
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            release_motor.send_cmd(
                sdk, serial, sdk.MotorCmd(), sdk.MotorData(), 2,
                mode=1, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0,
            )

    def test_fault_stop_never_uses_unvalidated_position_hold(self):
        class FakeRuntime:
            position_valid = False

            def __init__(self):
                self.motor_commands = 0
                self.stop_commands = 0

            def send_motor(self, *_args):
                self.motor_commands += 1

            def send_stop(self):
                self.stop_commands += 1
                return {"reply_valid": True, "merror": 0}

        controller = bridge.RealUnitreeLegBridge(
            motors_cfg=bridge.DEFAULT_MOTORS,
            enable_motors=True,
            dt=0.00001,
        )
        runtimes = {name: FakeRuntime() for name in bridge.MOTOR_NAMES}
        controller.runtime = runtimes
        result = controller.stop_all_immediately("bad startup read", fault=True)
        self.assertFalse(result["soft_release_attempted"])
        self.assertTrue(result["stop_reply_all_valid"])
        self.assertEqual(sum(rt.motor_commands for rt in runtimes.values()), 0)
        self.assertEqual(
            sum(rt.stop_commands for rt in runtimes.values()),
            bridge.STOP_REPEAT_COUNT * len(bridge.MOTOR_NAMES),
        )

    def test_homing_is_not_ready_until_measured_pose_is_stable(self):
        controller = bridge.RealUnitreeLegBridge(
            motors_cfg=bridge.DEFAULT_MOTORS,
            enable_motors=True,
        )
        now = time.time()
        controller.state = bridge.STATE_HOMING
        controller.homing_started_at = now - 2.0
        controller.homing_duration = 1.0
        controller.target_deg = {name: 0.0 for name in bridge.MOTOR_NAMES}
        controller.current_deg = {name: 10.0 for name in bridge.MOTOR_NAMES}
        self.assertFalse(controller.update_homing_state(now))
        self.assertFalse(controller.motors_ready)

        controller.current_deg = {name: 0.5 for name in bridge.MOTOR_NAMES}
        for _ in range(bridge.HOMING_STABLE_CYCLES - 1):
            self.assertFalse(controller.update_homing_state(now))
        self.assertTrue(controller.update_homing_state(now))
        self.assertTrue(controller.motors_ready)
        self.assertEqual(controller.state, bridge.STATE_READY)

    def test_homing_timeout_latches_failure(self):
        controller = bridge.RealUnitreeLegBridge(
            motors_cfg=bridge.DEFAULT_MOTORS,
            enable_motors=True,
        )
        controller.state = bridge.STATE_HOMING
        controller.homing_duration = 1.0
        controller.homing_started_at = 0.0
        controller.current_deg = {name: 20.0 for name in bridge.MOTOR_NAMES}
        with self.assertRaisesRegex(RuntimeError, "did not converge"):
            controller.update_homing_state(
                bridge.HOMING_TIMEOUT_MARGIN_SEC + 1.1
            )


if __name__ == "__main__":
    unittest.main()
