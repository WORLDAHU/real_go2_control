# 历史脚本归档

这里保存项目不同开发阶段留下的实验和一次性工具，用于追溯实现过程，不再作为日常运行入口。

## 目录

- `full_body_legacy/`：早期 GO2 整机 Pinocchio、MuJoCo、站立和跳跃实验。
- `single_leg_build_legacy/`：单腿导轨模型逐步生成、对齐和显示的旧流程。
- `hardware_bringup_legacy/`：电机方向、直接串口和独立小腿 home 的早期硬件摸底脚本。
- `model_migration_tools/`：曾用于修改 URDF/MJCF 的一次性迁移工具。

## 重要说明

这些文件通过 `git mv` 保留了历史，没有删除代码，但不保证能够在当前位置直接运行。
部分旧脚本使用：

```python
Path(__file__).resolve().parents[1]
```

移动到归档目录后，这个表达式不再指向项目根目录。如果确实需要复现实验，应先阅读代码、修正路径，
并检查它使用的模型、字段名、角度定义和依赖是否仍与当前项目一致。

归档脚本不得被 README、日常命令或网页模块当作正式接口依赖。当前正式入口见 `scripts/README.md`。
