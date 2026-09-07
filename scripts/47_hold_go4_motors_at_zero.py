#!/usr/bin/env python3
"""Speed-limited return of all ungeared GO4 motors, then hold until Ctrl+C."""
import argparse, json, math, os, sys, time
from pathlib import Path
SRC=Path(__file__).resolve().parents[1]/"src"; sys.path.insert(0,str(SRC))
from go4_leg_adapter import RL_MOTOR_ORDER
from unitree_daisy_chain import UnitreeDaisyChain, import_unitree_sdk, unwrap_near

def main():
 p=argparse.ArgumentParser(); p.add_argument("--reference",default=os.path.expanduser("~/go4_rl_reference.json")); p.add_argument("--sdk-path",default="/home/claww/unitree_actuator_sdk/lib")
 p.add_argument("--max-speed-deg-s",dest="max_speed",type=float,default=5.0); p.add_argument("--kp",type=float,default=.20); p.add_argument("--kd",type=float,default=.04); p.add_argument("--dt",type=float,default=.02); p.add_argument("--max-start-offset-deg",type=float,default=15.0); p.add_argument("--max-tracking-error-deg",type=float,default=2.0); p.add_argument("--enable-hold",action="store_true"); a=p.parse_args()
 if min(a.max_speed,a.dt,a.max_start_offset_deg,a.max_tracking_error_deg)<=0 or min(a.kp,a.kd)<0:p.error("invalid numeric option")
 ref=json.loads(Path(a.reference).expanduser().read_text()); motors=ref.get("motors",{}); ids=[int(motors[r]["id"]) for r in RL_MOTOR_ORDER]
 print(f"all-motor zero hold: max_speed={a.max_speed:.2f} output-deg/s kp={a.kp:.3f} kd={a.kd:.3f}")
 if not a.enable_hold: print("DRY RUN; add --enable-hold after checking support"); return 0
 bus=UnitreeDaisyChain(import_unitree_sdk(a.sdk_path),ref["port"]); gear=bus.gear_ratio(); last={}; result=0
 try:
  bus.stop_many(ids,repeats=3); start={}; zero={}; offsets={}
  for r in RL_MOTOR_ORDER:
   mid=int(motors[r]["id"]); start[r]=bus.read_mean_q(mid,samples=12); zero[r]=unwrap_near(float(motors[r]["q_reference_phase_rad"]),start[r]); offsets[r]=math.degrees(start[r]-zero[r])/gear; print(f"{r}: id={mid} offset={offsets[r]:+.3f} deg")
   if abs(offsets[r])>a.max_start_offset_deg: raise RuntimeError(f"{r} offset exceeds safe envelope")
  if input("Type HOLD to return all motors and hold zero: ").strip().upper()!="HOLD": print("Cancelled"); return 0
  duration=max(.5,max(abs(v) for v in offsets.values())/a.max_speed); begun=time.monotonic()
  while True:
   s=min((time.monotonic()-begun)/duration,1.0); b=.5-.5*math.cos(math.pi*s)
   for r in RL_MOTOR_ORDER:
    mid=int(motors[r]["id"]); cmd=start[r]+(zero[r]-start[r])*b; last[r]=unwrap_near(bus.transact(mid,q=cmd,dq=0,kp=a.kp,kd=a.kd,tau=0).q,cmd)
   if s>=1: break
   time.sleep(a.dt)
  print(f"zero reached in {duration:.2f}s; holding until Ctrl+C")
  bad=0
  while True:
   for r in RL_MOTOR_ORDER:
    mid=int(motors[r]["id"]); last[r]=unwrap_near(bus.transact(mid,q=zero[r],dq=0,kp=a.kp,kd=a.kd,tau=0).q,zero[r]); err=math.degrees(last[r]-zero[r])/gear
    bad=bad+1 if abs(err)>a.max_tracking_error_deg else 0
    if bad>=5: raise RuntimeError(f"{r} tracking error {err:+.2f} deg")
   time.sleep(a.dt)
 except KeyboardInterrupt: print("Hold interrupted; releasing smoothly.")
 except Exception as e: result=1; print(f"FAULT: {e}")
 finally:
  try:
   for i in range(30):
    scale=1-i/30
    for r in last: bus.transact(int(motors[r]["id"]),q=last[r],dq=0,kp=a.kp*scale,kd=a.kd*scale,tau=0,allow_fault=True)
    time.sleep(a.dt)
  except Exception as e: result=1; print(f"fade failed: {e}")
  try: bus.stop_many(ids,repeats=5); print("Stop replies confirmed for all IDs.")
  except Exception as e: result=1; print(f"STOP FAILED: {e}; cut power")
 return result
if __name__=="__main__": raise SystemExit(main())
