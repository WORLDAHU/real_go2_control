#!/usr/bin/env python3
"""Return one ungeared GO4 motor to its captured reference with a speed limit."""
import argparse, json, math, os, sys, time
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))
from go4_leg_adapter import RL_MOTOR_ORDER
from unitree_daisy_chain import UnitreeDaisyChain, import_unitree_sdk, unwrap_near

REF = os.path.expanduser("~/go4_rl_reference.json")

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--reference", default=REF); p.add_argument("--sdk-path", default="/home/claww/unitree_actuator_sdk/lib")
    p.add_argument("--motor", choices=RL_MOTOR_ORDER, required=True)
    p.add_argument("--max-speed-deg-s", type=float, default=5.0); p.add_argument("--kp",type=float,default=.20); p.add_argument("--kd",type=float,default=.04)
    p.add_argument("--hold-sec",type=float,default=2.0); p.add_argument("--dt",type=float,default=.02); p.add_argument("--max-start-offset-deg",type=float,default=15.0)
    p.add_argument("--enable-motion",action="store_true"); a=p.parse_args()
    if min(a.max_speed_deg_s,a.dt,a.max_start_offset_deg)<=0 or min(a.kp,a.kd,a.hold_sec)<0: p.error("invalid numeric option")
    ref=json.loads(Path(a.reference).expanduser().read_text(encoding="utf-8")); motors=ref.get("motors",{})
    if ref.get("schema")!="go4_rl_relative_reference_v1" or a.motor not in motors: raise SystemExit("invalid reference; run script 43")
    ids=[int(motors[r]["id"]) for r in RL_MOTOR_ORDER]; motor_id=int(motors[a.motor]["id"])
    print(f"motor={a.motor} id={motor_id} max_speed={a.max_speed_deg_s:.2f} output-deg/s")
    if not a.enable_motion: print("DRY RUN; add --enable-motion to return to zero"); return 0
    bus=UnitreeDaisyChain(import_unitree_sdk(a.sdk_path),ref["port"]); gear=bus.gear_ratio(); result=0
    try:
        bus.stop_many(ids,repeats=3); q0=bus.read_mean_q(motor_id,samples=15); qz=unwrap_near(float(motors[a.motor]["q_reference_phase_rad"]),q0)
        offset=math.degrees(q0-qz)/gear
        print(f"current offset={offset:+.3f} deg; zero rotor={qz:+.6f} rad")
        if abs(offset)>a.max_start_offset_deg: raise RuntimeError("offset exceeds safe single-turn return envelope")
        if input("Type ZERO to return this motor: ").strip().upper()!="ZERO": print("Cancelled"); return 0
        duration=max(.5,abs(offset)/a.max_speed_deg_s); start=time.monotonic(); last=q0
        while True:
            s=min((time.monotonic()-start)/duration,1.0); blend=.5-.5*math.cos(math.pi*s); cmd=q0+(qz-q0)*blend
            reply=bus.transact(motor_id,q=cmd,dq=0,kp=a.kp,kd=a.kd,tau=0); last=unwrap_near(reply.q,cmd)
            if s>=1: break
            time.sleep(a.dt)
        end=time.monotonic()+a.hold_sec
        while time.monotonic()<end: last=unwrap_near(bus.transact(motor_id,q=qz,dq=0,kp=a.kp,kd=a.kd,tau=0).q,qz); time.sleep(a.dt)
        error=math.degrees(last-qz)/gear; print(f"zero reached: error={error:+.3f} deg, duration={duration:.2f}s")
        if abs(error)>.5: raise RuntimeError("final zero error exceeds 0.5 deg")
    except Exception as e: result=1; print(f"FAULT: {e}")
    finally:
        try: bus.stop_many(ids,repeats=5); print("Stop replies confirmed for all IDs.")
        except Exception as e: result=1; print(f"STOP FAILED: {e}; cut power")
    return result
if __name__=="__main__": raise SystemExit(main())
