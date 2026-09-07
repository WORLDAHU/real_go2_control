#!/usr/bin/env python3
"""Run a GO4 RL extension-equivalent motor motion without installed gears."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from go4_leg_adapter import RL_MOTOR_ORDER, joint_delta_to_motor_output_delta_deg
from go4_rl_kinematics import Go4RLKinematics
from unitree_daisy_chain import UnitreeDaisyChain, import_unitree_sdk, unwrap_near

URDF = ROOT / "models" / "go4" / "GO4.urdf"
REFERENCE = os.path.expanduser("~/go4_rl_reference.json")
DIRECTIONS = {"hip": -1.0, "thigh": 1.0, "knee": -1.0}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--reference", default=REFERENCE)
    p.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    p.add_argument("--stroke-mm", type=float, default=10.0)
    p.add_argument("--segment-sec", type=float, default=3.0)
    p.add_argument("--max-speed-deg-s", type=float, default=5.0)
    p.add_argument("--kp", type=float, default=0.20)
    p.add_argument("--kd", type=float, default=0.04)
    p.add_argument("--hip-kp", type=float, help="Override kp for hip only")
    p.add_argument("--hip-kd", type=float, help="Override kd for hip only")
    p.add_argument("--dt", type=float, default=0.02)
    p.add_argument("--max-start-offset-deg", type=float, default=3.0)
    p.add_argument("--max-tracking-error-deg", type=float, default=2.0)
    p.add_argument("--prehold-sec", type=float, default=3.0)
    p.add_argument("--gears-not-installed", action="store_true")
    p.add_argument("--shafts-free", action="store_true")
    p.add_argument("--enable-motion", action="store_true")
    a = p.parse_args()
    if not 0.0 < a.stroke_mm <= 60.0:
        p.error("bare demo stroke must be in (0, 60] mm")
    if min(a.segment_sec, a.max_speed_deg_s, a.dt, a.max_start_offset_deg, a.max_tracking_error_deg, a.prehold_sec) <= 0:
        p.error("timing, speed and tolerances must be positive")
    if a.hip_kp is not None and a.hip_kp < 0 or a.hip_kd is not None and a.hip_kd < 0:
        p.error("hip kp/kd must be non-negative")
    kp = {role: a.kp for role in RL_MOTOR_ORDER}
    kd = {role: a.kd for role in RL_MOTOR_ORDER}
    kp["hip"] = a.kp if a.hip_kp is None else a.hip_kp
    kd["hip"] = a.kd if a.hip_kd is None else a.hip_kd

    model = Go4RLKinematics.from_urdf(URDF)
    trajectory = model.extension_cycle(a.stroke_mm / 1000.0, samples_per_leg=30)
    start_joint = np.degrees(trajectory[0])
    bottom_joint = np.degrees(trajectory[len(trajectory) // 2])
    joint_delta = dict(zip(RL_MOTOR_ORDER, bottom_joint - start_joint))
    motor_delta = {
        role: joint_delta_to_motor_output_delta_deg(role, joint_delta[role], DIRECTIONS[role])
        for role in RL_MOTOR_ORDER
    }
    peak_delta = max(abs(value) for value in motor_delta.values())
    segment_sec = max(a.segment_sec, peak_delta * math.pi / (2.0 * a.max_speed_deg_s))
    print(f"bare GO4 RL equivalent demo: stroke={a.stroke_mm:.1f} mm")
    for role in RL_MOTOR_ORDER:
        print(f"{role}: joint delta={joint_delta[role]:+.3f} deg, motor output={motor_delta[role]:+.3f} deg")
    print(f"segment={segment_sec:.2f}s, peak speed <= {a.max_speed_deg_s:.2f} output-deg/s")
    print(f"gains: hip kp/kd={kp['hip']:.3f}/{kd['hip']:.3f}, thigh/knee={a.kp:.3f}/{a.kd:.3f}")
    if not (a.gears_not_installed and a.shafts_free and a.enable_motion):
        print("DRY RUN. Motion requires --gears-not-installed --shafts-free --enable-motion")
        return 0

    ref = json.loads(Path(a.reference).expanduser().read_text(encoding="utf-8"))
    motors = ref["motors"]
    ids = [int(motors[role]["id"]) for role in RL_MOTOR_ORDER]
    bus = UnitreeDaisyChain(import_unitree_sdk(a.sdk_path), ref["port"])
    gear = bus.gear_ratio()
    zero = {}
    current = {}
    last = {}
    result = 0
    try:
        print("Support the free shafts, then confirm. Holding starts immediately after confirmation.")
        if input("Type HOLD_DEMO to acquire, hold, return, then run: ").strip().upper() != "HOLD_DEMO":
            print("Cancelled")
            return 0

        bus.stop_many(ids, repeats=3)
        # Take one fast snapshot of every free shaft, then immediately hold the
        # captured positions.  The old multi-sample, zero-stiffness reads let a
        # gravity-loaded hip sag while the operator was typing and while the
        # other motors were being sampled.
        for role in RL_MOTOR_ORDER:
            motor_id = int(motors[role]["id"])
            current[role] = bus.query(motor_id).q

        for _ in range(12):
            for role in RL_MOTOR_ORDER:
                motor_id = int(motors[role]["id"])
                last[role] = unwrap_near(
                    bus.transact(
                        motor_id,
                        q=current[role],
                        dq=0,
                        kp=kp[role],
                        kd=kd[role],
                        tau=0,
                    ).q,
                    current[role],
                )
            time.sleep(a.dt)

        current = dict(last)
        for role in RL_MOTOR_ORDER:
            zero[role] = unwrap_near(float(motors[role]["q_reference_phase_rad"]), current[role])
            offset = math.degrees(current[role] - zero[role]) / gear
            print(f"{role}: start offset={offset:+.3f} output deg")
            if abs(offset) > a.max_start_offset_deg:
                raise RuntimeError(f"{role} is not close enough to motor zero")

        targets = [
            dict(zero),
            {role: zero[role] + math.radians(motor_delta[role]) * gear for role in RL_MOTOR_ORDER},
            dict(zero),
        ]
        starts = dict(current)
        excessive = {role: 0 for role in RL_MOTOR_ORDER}
        for target_index, target in enumerate(targets):
            distance_deg = max(
                abs(math.degrees(target[role] - starts[role]) / gear)
                for role in RL_MOTOR_ORDER
            )
            actual_segment_sec = max(
                segment_sec,
                distance_deg * math.pi / (2.0 * a.max_speed_deg_s),
            )
            begun = time.monotonic()
            while True:
                s = min((time.monotonic() - begun) / actual_segment_sec, 1.0)
                blend = 0.5 - 0.5 * math.cos(math.pi * s)
                for role in RL_MOTOR_ORDER:
                    cmd = starts[role] + (target[role] - starts[role]) * blend
                    motor_id = int(motors[role]["id"])
                    last[role] = unwrap_near(bus.transact(motor_id, q=cmd, dq=0, kp=kp[role], kd=kd[role], tau=0).q, cmd)
                    error = abs(math.degrees(last[role] - cmd) / gear)
                    excessive[role] = excessive[role] + 1 if error > a.max_tracking_error_deg else 0
                    if excessive[role] >= 5:
                        raise RuntimeError(f"{role} tracking error={error:.2f} deg")
                if s >= 1.0:
                    break
                time.sleep(a.dt)
            starts = dict(last)
            if target_index == 0:
                print(f"All motors at zero; holding {a.prehold_sec:.1f}s before demo. You may release support.")
                until = time.monotonic() + a.prehold_sec
                while time.monotonic() < until:
                    for role in RL_MOTOR_ORDER:
                        motor_id = int(motors[role]["id"])
                        last[role] = unwrap_near(
                            bus.transact(motor_id, q=zero[role], dq=0, kp=kp[role], kd=kd[role], tau=0).q,
                            zero[role],
                        )
                    time.sleep(a.dt)
        print(f"{a.stroke_mm:.1f} mm equivalent completed and all motors returned to zero.")
    except KeyboardInterrupt:
        result = 130
        print("Interrupted")
    except Exception as exc:
        result = 1
        print(f"FAULT: {exc}")
    finally:
        try:
            bus.stop_many(ids, repeats=5)
            print("Stop replies confirmed for all IDs.")
        except Exception as exc:
            result = 1
            print(f"STOP FAILED: {exc}; cut power")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
