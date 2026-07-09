'''
零点要运行：


/home/claww/miniforge3/envs/go2-convex-mpc/bin/python scripts/33_calibrate_motor_home.py \
  --sdk-path /home/claww/unitree_actuator_sdk/lib

并输入大写：
YES


启动 bridge：

/home/claww/miniforge3/envs/go2-convex-mpc/bin/python scripts/32_real_unitree_leg_bridge.py \
  --sdk-path /home/claww/unitree_actuator_sdk/lib \
  --enable-motors \
  --dt 0.02 \
  --ramp-time 0.05 \
  --kp 0.2 \
  --kd 0.02


另一个终端跑：
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python scripts/38_real_leg_pose_sequence.py \
  --preset micro \
  --dt 0.05









'''