# 早期硬件摸底脚本

本目录保存电机方向验证、直接 Unitree 电机访问和独立小腿角度测试的早期脚本。

这些脚本不再是正式实机入口：

- `31_test_single_motor_direction.py`：电机方向已经确认，当前不再需要日常运行。
- `34_test_single_unitree_motor_direct.py`：直接打开串口，缺少当前完整的统一 home 语义。
- `35_single_calf_motor_angle.py`：使用旧的 `~/single_calf_home.json`，已被统一读取 `~/motor_home.json` 的
  `scripts/37_test_leg_motor_angle.py` 取代。

## 安全警告

默认不要运行本目录脚本。若为追溯旧实验必须运行，至少先确认：

1. `scripts/32_real_unitree_leg_bridge.py` 已完全退出。
2. 没有其他进程打开相同 `/dev/ttyUSB*`。
3. 当前电机 ID、端口、方向、限位和标定文件语义已经重新核对。
4. 腿部路径安全，并准备好断电和释放措施。

正式标定、单电机测试和释放分别使用当前的 `33`、`37`、`39` 脚本。
