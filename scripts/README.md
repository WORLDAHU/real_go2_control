# 当前脚本入口

本目录只保留当前单腿导轨仿真、实机角度适配和三电机测试仍在使用的入口。
历史整机实验、旧单腿模型生成步骤、硬件摸底脚本和一次性模型修复工具已经移动到
`archive/scripts/`。

## Python 环境

运行本项目脚本时使用：

```bash
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python
```

不要使用 `/usr/bin/python3`，也不要使用 `sudo python3`。

## 当前单腿仿真与角度适配

- `25_single_leg_slider_footspace_view.py`：单腿导轨模型生成、足端轨迹、IK、PD 和 Viewer 综合入口。
- `26_view_single_leg_control_only.py`：读取现有 MJCF，运行当前单腿控制器并显示。
- `27_dry_run_real_leg_commands.py`：离线检查仿真目标到 common/bridge 三层角度的转换。
- `28_mock_real_leg_bridge.py`：不访问串口的 HTTP 假 bridge，用于接口测试。
- `29_stream_real_leg_commands.py`：将单腿控制器输出适配后发送给 bridge；使用 HTTP 模式前必须确认目标安全。
- `30_check_common_joint_angles.py`：离线检查 sim、common 和 bridge 三层角度。

## 当前实机工具

- `32_real_unitree_leg_bridge.py`：唯一正式三电机 bridge 和 HTTP 控制入口。
- `33_calibrate_motor_home.py`：唯一正式固定姿态编码器参考标定入口，保存 `~/motor_home.json`。
- `36_verify_calf_fourbar_mapping.py`：小腿四连杆纯数学验证，不访问串口。
- `37_test_leg_motor_angle.py`：读取统一 home 文件的单电机角度测试。
- `38_real_leg_pose_sequence.py`：通过现有 bridge 发送三电机平滑姿态序列。
- `39_release_leg_motors.py`：先零刚度再停止模式的电机释放工具。
- `40_record_leg_bridge_status.py`：只读记录 `/status` 中的目标、实际角和跟踪误差。

## GO4 菊花链裸电机测试（新硬件）

GO4 使用一个 USB/RS485 端口依次访问多台不同 ID 的电机。机械顺序统一写成
`hip -> thigh -> knee`，默认暂定 ID 为 `0 -> 1 -> 2`。物理串接顺序不能证明
电机 ID，第一次必须先扫描：

```bash
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/41_scan_daisy_chain.py \
  --sdk-path /home/claww/unitree_actuator_sdk/lib \
  --port /dev/ttyUSB0 \
  --ids 0 1 2
```

`41` 只发送零刚度、零力矩查询。确认 ID 后先 dry-run 一台裸电机：

```bash
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/42_test_one_bare_motor.py --motor hip
```

外部 16:28 齿轮和平行四边形断开、电机输出轴可自由转动，并确认角色与 ID 后，
才允许显式打开动作：

```bash
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/42_test_one_bare_motor.py \
  --motor hip --motor-id 0 --enable-motion
```

默认动作是当前输出位置附近 `+2° -> 0° -> -2° -> 0°`，不是 URDF 关节角。
`src/go4_leg_adapter.py` 已提供电机输出角与膝关节角之间的 16:28 换算，但在
传动机构装好、方向和机械零位实测前，`42` 不使用这项换算。

三台裸电机测试通过后，安装传动并支撑整条腿。`43` 只读捕获一个无碰撞的
临时参考姿态；该文件不是 URDF 机械零位：

```bash
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/43_capture_go4_rl_reference.py
```

随后可先 dry-run 检查某个装配关节的相对 ±1° 命令。必须明确传入方向，膝关节
会自动将 1° 关节运动换算为 1.75° 电机输出运动：

```bash
/home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
  scripts/44_test_go4_rl_joint_relative.py \
  --joint knee --direction 1
```

真实运动还必须同时提供 `--transmission-installed --joint-supported --enable-motion`。
另外两个关节只会收到 STOP，所以必须由台架支撑或机械约束，不能依靠它们主动
保持姿态。首次装配测试硬限制为相对参考姿态 ±2°。

## 实机并发安全规则

1. 同一时间只能有一个程序打开某个电机串口。
2. 运行 `32_real_unitree_leg_bridge.py` 时，不得同时运行 `33`、`37` 或 `39`。
3. HTTP 端口不同不代表电机访问互不冲突；多个进程打开相同 USB/RS485 仍会抢占电机。
4. `28_mock_real_leg_bridge.py` 不访问串口，但不要让它与真实 bridge 绑定同一个 HTTP 端口。
5. `33`、`37`、`39` 直接访问串口，运行前必须确认真实 bridge 已完全退出。
6. 正式固定姿态标定只使用 `33_calibrate_motor_home.py`，不要恢复旧的独立小腿 home 文件。
7. GO4 菊花链只允许一个进程创建一个 `SerialPort`；不得与旧 `32/33/37/39` 同时运行。

## 固定标定姿态与编码器类型

GO-M8010-6 使用转子侧单圈绝对式编码器。单圈相位是绝对的，但电机断电后
固件不保留累计圈数，因此同一姿态的 `data.q` 可能相差整数个 `2π`。Bridge
会对齐这个累计圈数分支；它不能替代多圈绝对或输出轴绝对传感器。

`bridge_cmd_deg = 0` 表示回到固定标定姿态，不表示三个 common 机械角全部为零：

- `hip_motor = 0°`：`common hip_abduction = 0°`，髋无内外摆动。
- `thigh_motor = 0°`：`common thigh_pitch = +90°`，大腿水平。
- `calf_motor = 0°`：曲柄为 `10°`，`common knee_pitch` 由当前四连杆几何自动计算（当前约 `-160.623°`），小腿完全收缩。

更完整的命令和操作说明见 `src/00_some_scripts.py`。
