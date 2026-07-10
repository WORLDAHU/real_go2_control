"""
GO2 左后腿实机常用命令
=========================

这是实机台架操作命令备忘录，不是控制程序；复制各段命令到 WSL / 电机电脑
终端执行即可。所有 Python 命令必须使用：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python

不要使用 /usr/bin/python3。执行前进入项目目录：

    cd ~/Dev/Projects/MPC/real_go2_control


============================================================================
一、先理解四种坐标/零点
============================================================================

1. 电机编码器 q（rad）
   GO-M8010-6 返回的原始转子位置 data.q，不能直接当成大腿或膝盖机械角。

2. q_home：上电标定的编码器参考（rad）
   在固定人工标定姿态读取的 q。它不是电机出厂零点，也不是所有关节的
   common 机械零位。bridge 的底层换算为：

       q_target = q_home + radians(bridge_cmd_deg * direction) * gear_ratio

   所以 bridge_cmd_deg=0 的含义是“回到本次上电标定姿态”。

3. common_joint_deg（deg）
   仿真、RL、运动学使用的固定机械坐标；不会因为每次重新上电而改变。

4. bridge_cmd_deg（deg）
   最终发送给 bridge 的坐标；相对本次上电标定姿态计数。

每次运行 33 标定脚本前，先释放电机，然后手动摆到同一套固定姿态：

    hip_motor：无内外摆动
        common hip_abduction =   0.00 deg
        bridge hip_motor    =   0.00 deg

    thigh_motor：大腿水平
        common thigh_pitch  = +90.00 deg
        bridge thigh_motor  =   0.00 deg

    calf_motor：小腿完全收缩的四连杆限位
        crank_angle         = +10.00 deg
        common knee_pitch   = -160.59 deg
        bridge calf_motor   =   0.00 deg

大腿最容易混淆：大腿水平是上电标定姿态，不是 common thigh 的机械零位。

    bridge_thigh_cmd = common_thigh_pitch - 90 deg

例如 common +90 deg（水平）对应 bridge 0；common 0 deg 对应 bridge -90。
大腿当前 bridge 安全范围为 [-120, 0] deg。

小腿不能线性做 knee 偏置，必须通过 real_leg_adapter.py 中的四连杆反解；
calf bridge=0 已经对应 crank=10 deg、knee 约 -160.59 deg。


============================================================================
二、释放电机：手动摆腿或重新标定前的第一步
============================================================================

释放全部电机。脚本会先保持当前读数、逐步降刚度，再发 mode=0，避免错误
地向 q=0 运动：
如果之前开了bride，先停止 bridge。它会安全降刚度并 stop

    curl -X POST http://127.0.0.1:8765/stop

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/39_release_leg_motors.py --motor all

仅释放大腿：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/39_release_leg_motors.py --motor thigh_motor

电机变松是预期行为；释放后才能安全地手动摆到固定标定姿态。


============================================================================
三、每次上电后的 q_home 标定
============================================================================

确认没有运行带 --enable-motors 的 bridge；释放电机并摆到第一节姿态后运行：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/33_calibrate_motor_home.py \
      --sdk-path /home/claww/unitree_actuator_sdk/lib

确认姿态正确后输入大写：

    YES

它会覆写 ~/motor_home.json，并记录 q_home 对应的固定机械姿态元数据。32 与
37 会拒绝缺少该元数据的旧 home 文件；这是安全检查，不能忽略。


============================================================================
四、纯软件映射检查：不访问串口，不控制电机
============================================================================

修改 adapter 或重新确认坐标后先运行：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/30_check_common_joint_angles.py

输出中应有：

    thigh_motor = common thigh_pitch - 90 deg


============================================================================
五、单电机小范围方向检查
============================================================================

每次只测试一个电机；支撑腿部，使用低 kp/kd，确认方向后再扩大范围。脚本
结束会自动软释放该电机。

避免 bridge 运行时执行 37；同一电机只能由一个进程控制。
先关闭bridge，

    curl -X POST http://127.0.0.1:8765/stop


髋：bridge +5 deg：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/37_test_leg_motor_angle.py \
      --motor hip_motor --angle-deg 5 --kp 0.25 --kd 0.03 --hold-sec 2


  



大腿：bridge -5 deg。它使 common 大腿从标定的 +90 变成 +85。不要沿用旧
约定下的 +5 deg：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/37_test_leg_motor_angle.py \
      --motor thigh_motor --angle-deg -5 --kp 0.25 --kd 0.03 --hold-sec 2

小腿：bridge -20 deg。这不是 knee=-20，而是相对收缩标定姿态的小齿轮/电机
命令：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/37_test_leg_motor_angle.py \
      --motor calf_motor --angle-deg 0 --kp 0.25 --kd 0.03 --hold-sec 2


============================================================================
六、启动三电机 bridge
============================================================================

完成第三节标定后启动。bridge 首先会抢占（绑定）HTTP 端口 8765；若该端口
已经被旧 bridge 占用，新进程会立即退出，且不会打开电机串口、不会读取电机、
不会执行归位。这是为了阻止两个 bridge 同时向同一组 USB/RS485 发命令。

端口可用时，bridge 才会读取并校验 ~/motor_home.json，显示每台电机上次标定
时间与固定姿态；只有输入大写 YES，才会自动慢速回到本次 q_home 对应的 bridge
0/0/0 标定姿态。若没有文件、姿态元数据不匹配或未输入 YES，bridge 不会开始
控制；应重新运行第三节的 33 标定。

如果启动提示 "Address already in use"，说明通常已有 bridge 在监听 8765。
不要启动第二个 bridge，也不要运行 37；先停止旧 bridge：

    curl -X POST http://127.0.0.1:8765/stop

确认端口已空闲：

    ss -ltnp '( sport = :8765 )'

三根 USB/RS485 推荐 50 Hz。普通连续轨迹的 ramp-time 保持小；首次归位使用
独立的 3 秒 home-ramp-time，避免把一大段回位压缩到 0.05 秒：

/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/32_real_unitree_leg_bridge.py \
  --sdk-path /home/claww/unitree_actuator_sdk/lib \
  --enable-motors \
  --dt 0.02 \
  --ramp-time 0.05 \
  --home-ramp-time 3.0 \
  --kp 0.25 \
  --kd 0.03
  
更改刚度

出现启动确认提示后，先确认腿部回标定姿态的路径无碰撞，再输入：

    YES

若有 CRC、timeout、短帧或电机无回应：先停止 bridge、检查接线，再将 bridge
的 --dt 改慢到 0.03 或 0.05。38 的 --dt 是 HTTP 发送周期，不是通信周期。


============================================================================
七、bridge HTTP 状态、保守命令与停止
============================================================================

另一个终端查看目标、当前读数和错误：

    curl http://127.0.0.1:8765/status

回到固定标定姿态：髋中位、大腿水平、小腿完全收缩。它可能让腿运动到小腿
收缩限位，因此仅在确认路径安全时使用：

    curl -X POST http://127.0.0.1:8765/set_motor_commands \
      -H "Content-Type: application/json" \
      -d '{"hip_motor": 0, "thigh_motor": 0, "calf_motor": 0}'

相对更保守的起始姿态：髋中位、大腿水平、小腿从收缩限位稍微放开：

    curl -X POST http://127.0.0.1:8765/set_motor_commands \
      -H "Content-Type: application/json" \
      -d '{"hip_motor": 0, "thigh_motor": 0, "calf_motor": -20}'

停止 bridge。它会安全降刚度并 stop；若随后要手动摆腿，再运行第二节 39：

    curl -X POST http://127.0.0.1:8765/stop


============================================================================
八、慢速三电机关联测试
============================================================================

bridge 正在运行且 /status 显示 motors_ready=true 时，在第二终端先跑 micro。
38 的 hip/thigh/calf 全部是 bridge 坐标；新约定下 thigh 必须为非正数。

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/38_real_leg_pose_sequence.py --preset micro --dt 0.05

仅在 micro 的方向、平滑性和机械余量全部干净时，再运行：


/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/38_real_leg_pose_sequence.py \
  --preset push_preview \
  --dt 0.05 \
  --repeat 3

  重复两次

============================================================================
九、推荐安全顺序
============================================================================

    1. 39_release_leg_motors.py --motor all
    2. 手动摆到固定标定姿态
    3. 33_calibrate_motor_home.py，并确认 YES
    4. 30_check_common_joint_angles.py（纯软件检查）
    5. 37 单电机小范围测试
    6. 32 启动 bridge
    7. /status 确认正常
    8. 38 --preset micro
    9. /stop；必要时再运行 39 释放

异常运动、通信错误、方向不确定或碰限位时：立即停止进一步动作，先 POST
/stop；需要手动处理时，再运行 39 释放电机。
"""
