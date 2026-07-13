# 旧单腿模型生成流程

本目录保存将整机 URDF 裁成 RL 左后腿、生成 MuJoCo MJCF、添加视觉网格、对齐导轨和运行早期
单腿跳跃示例的逐步脚本。

当前综合入口为：

```text
scripts/25_single_leg_slider_footspace_view.py
```

当前控制器查看入口为：

```text
scripts/26_view_single_leg_control_only.py
```

需要重新生成 `models/single_leg/RL_single_leg_slider_mujoco.xml` 时，应优先检查并使用当前综合入口，
不要同时混用本目录的旧分步脚本。

本目录脚本的相对路径假设可能因归档而失效，运行前必须先检查 `PROJECT_ROOT` 和输出路径。
