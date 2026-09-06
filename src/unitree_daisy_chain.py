"""Minimal, safety-oriented Unitree RS485 daisy-chain helpers.

One :class:`UnitreeDaisyChain` owns one serial adapter.  Motors on the bus are
addressed by ID and transactions are deliberately serialized; callers must not
open the same adapter from another process at the same time.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Iterable, Optional


MAX_ABS_ROTOR_RAD = 10000.0
TWO_PI = 2.0 * math.pi


def import_unitree_sdk(sdk_path: Optional[str] = None):
    if sdk_path:
        resolved = str(Path(sdk_path).expanduser().resolve())
        if resolved not in sys.path:
            sys.path.insert(0, resolved)
    import unitree_actuator_sdk as sdk

    return sdk


def unwrap_near(angle_rad: float, reference_rad: float) -> float:
    """Put a single-turn rotor angle on the 2*pi branch nearest reference."""

    return float(angle_rad) + round(
        (float(reference_rad) - float(angle_rad)) / TWO_PI
    ) * TWO_PI


@dataclass(frozen=True)
class MotorReply:
    motor_id: int
    q: float
    dq: float
    merror: int


class UnitreeDaisyChain:
    """Sequential request/reply access to several motors on one RS485 port."""

    def __init__(self, sdk, port: str):
        self.sdk = sdk
        self.port = str(port)
        self.serial = sdk.SerialPort(self.port)
        self.motor_type = sdk.MotorType.GO_M8010_6
        self.foc_mode = sdk.queryMotorMode(
            self.motor_type, sdk.MotorMode.FOC
        )

    def gear_ratio(self) -> float:
        ratio = float(self.sdk.queryGearRatio(self.motor_type))
        if not math.isfinite(ratio) or ratio <= 0.0:
            raise RuntimeError(f"invalid SDK gear ratio: {ratio!r}")
        return ratio

    def _new_frame(self, motor_id: int):
        cmd = self.sdk.MotorCmd()
        data = self.sdk.MotorData()
        cmd.motorType = self.motor_type
        data.motorType = self.motor_type
        cmd.id = int(motor_id)
        return cmd, data

    def transact(
        self,
        motor_id: int,
        *,
        q: float = 0.0,
        dq: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        tau: float = 0.0,
        allow_fault: bool = False,
    ) -> MotorReply:
        values = (q, dq, kp, kd, tau)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("motor command contains a non-finite value")

        cmd, data = self._new_frame(motor_id)
        cmd.mode = self.foc_mode
        cmd.q = float(q)
        cmd.dq = float(dq)
        cmd.kp = float(kp)
        cmd.kd = float(kd)
        cmd.tau = float(tau)

        ok = bool(self.serial.sendRecv(cmd, data))
        if not ok:
            raise RuntimeError(f"motor id={motor_id} timeout/no reply")
        if not bool(data.correct):
            raise RuntimeError(f"motor id={motor_id} invalid CRC/frame")
        if int(data.motor_id) != int(motor_id):
            raise RuntimeError(
                f"reply id={int(data.motor_id)}, expected id={int(motor_id)}"
            )

        rotor_q = float(data.q)
        if not math.isfinite(rotor_q) or abs(rotor_q) > MAX_ABS_ROTOR_RAD:
            raise RuntimeError(f"motor id={motor_id} invalid rotor q={rotor_q!r}")
        merror = int(data.merror)
        if merror != 0 and not allow_fault:
            raise RuntimeError(f"motor id={motor_id} merror={merror}")

        return MotorReply(
            motor_id=int(motor_id),
            q=rotor_q,
            dq=float(data.dq),
            merror=merror,
        )

    def query(self, motor_id: int, *, allow_fault: bool = False) -> MotorReply:
        """Read with zero position stiffness and zero feed-forward torque."""

        return self.transact(
            motor_id,
            q=0.0,
            dq=0.0,
            kp=0.0,
            kd=0.0,
            tau=0.0,
            allow_fault=allow_fault,
        )

    def read_mean_q(
        self,
        motor_id: int,
        *,
        samples: int = 10,
        interval_sec: float = 0.01,
        max_spread_rad: float = 0.10,
    ) -> float:
        if samples < 1:
            raise ValueError("samples must be >= 1")
        readings = []
        for _ in range(samples):
            reply = self.query(motor_id)
            q = reply.q if not readings else unwrap_near(reply.q, readings[-1])
            readings.append(q)
            if interval_sec > 0.0:
                time.sleep(interval_sec)
        spread = max(readings) - min(readings)
        if spread > max_spread_rad:
            raise RuntimeError(
                f"motor id={motor_id} unstable rotor readings: "
                f"spread={spread:.6f} rad > {max_spread_rad:.6f} rad"
            )
        return sum(readings) / len(readings)

    def stop(self, motor_id: int, *, allow_fault: bool = True) -> MotorReply:
        cmd, data = self._new_frame(motor_id)
        stop_mode = getattr(self.sdk.MotorMode, "STOP", None)
        if stop_mode is None:
            cmd.mode = 0
        else:
            try:
                cmd.mode = self.sdk.queryMotorMode(self.motor_type, stop_mode)
            except Exception:
                cmd.mode = 0
        cmd.q = 0.0
        cmd.dq = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.tau = 0.0

        ok = bool(self.serial.sendRecv(cmd, data))
        if not ok:
            raise RuntimeError(f"motor id={motor_id} stop timeout/no reply")
        if not bool(data.correct) or int(data.motor_id) != int(motor_id):
            raise RuntimeError(f"motor id={motor_id} invalid stop reply")
        reply = MotorReply(
            motor_id=int(motor_id),
            q=float(data.q),
            dq=float(data.dq),
            merror=int(data.merror),
        )
        if reply.merror != 0 and not allow_fault:
            raise RuntimeError(f"motor id={motor_id} stop merror={reply.merror}")
        return reply

    def stop_many(self, motor_ids: Iterable[int], *, repeats: int = 3) -> None:
        errors = []
        for _ in range(max(1, int(repeats))):
            for motor_id in motor_ids:
                try:
                    self.stop(int(motor_id))
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            raise RuntimeError("; ".join(errors[-3:]))
