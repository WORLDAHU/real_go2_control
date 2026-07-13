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
- `33_calibrate_motor_home.py`：唯一正式上电零点标定入口，保存 `~/motor_home.json`。
- `36_verify_calf_fourbar_mapping.py`：小腿四连杆纯数学验证，不访问串口。
- `37_test_leg_motor_angle.py`：读取统一 home 文件的单电机角度测试。
- `38_real_leg_pose_sequence.py`：通过现有 bridge 发送三电机平滑姿态序列。
- `39_release_leg_motors.py`：先零刚度再停止模式的电机释放工具。
- `40_record_leg_bridge_status.py`：只读记录 `/status` 中的目标、实际角和跟踪误差。

## 实机并发安全规则

1. 同一时间只能有一个程序打开某个电机串口。
2. 运行 `32_real_unitree_leg_bridge.py` 时，不得同时运行 `33`、`37` 或 `39`。
3. HTTP 端口不同不代表电机访问互不冲突；多个进程打开相同 USB/RS485 仍会抢占电机。
4. `28_mock_real_leg_bridge.py` 不访问串口，但不要让它与真实 bridge 绑定同一个 HTTP 端口。
5. `33`、`37`、`39` 直接访问串口，运行前必须确认真实 bridge 已完全退出。
6. 正式上电标定只使用 `33_calibrate_motor_home.py`，不要恢复旧的独立小腿 home 文件。

## 固定上电标定姿态

`bridge_cmd_deg = 0` 表示回到固定上电标定姿态，不表示三个 common 机械角全部为零：

- `hip_motor = 0°`：`common hip_abduction = 0°`，髋无内外摆动。
- `thigh_motor = 0°`：`common thigh_pitch = +90°`，大腿水平。
- `calf_motor = 0°`：曲柄为 `10°`，`common knee_pitch` 由当前四连杆几何自动计算（当前约 `-160.623°`），小腿完全收缩。

更完整的命令和操作说明见 `src/00_some_scripts.py`。
