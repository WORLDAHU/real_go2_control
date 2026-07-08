# real_go2_control 快速复习笔记

这份笔记用于快速回顾当前项目已经做过的内容，以及后续如何继续使用。

## 1. WSL、Ubuntu、挂载路径

进入 Ubuntu：



判断当前在哪个系统路径：

/mnt/c/Users/claww/Dev/Projects/MPC/go2-convex-mpc

常见路径区别：



比如：



实际对应 Windows：



当前学习项目使用 Ubuntu 自己的路径：



进入项目：



## 2. Python 环境

不要用系统 Python：



它可能没有 
umpy、pinocchio、mujoco。

使用 conda 环境：



检查当前 Python：



正确时应接近：



之后运行脚本可直接用：



## 3. 当前项目结构



## 4. URDF 和 Pinocchio

URDF 路径：



eal_go2_model.py 负责加载 URDF：



oot_joint=pin.JointModelFreeFlyer() 表示机器人基座是自由浮动的。

Pinocchio 输出里的：



含义：



检查模型：



关键正确输出：



## 5. q 和 dq

q 是广义位置，长度 19：



dq 是广义速度，长度 18：



obot_state.py 把这些数组拆成可读字段：



## 6. 足端位置

输出示例：



含义：



前后绝对值不完全一样，通常是因为 ase_link 原点不在几何中心，或者 CAD/URDF 导出存在微小偏差。

## 7. Jacobian 和关节力矩

核心公式：



含义：



leg_controller.py 做两件事：



检查四条腿力矩：



正确时：



注意：代码里的 orce_world 可以理解为地面对脚的力。真实物理中，脚向下压地，地面向上推脚，二者大小相等、方向相反。

## 8. 站立控制器

standing_controller.py 根据机器人质量自动分配支撑力：



检查：



正确时：



## 9. 竖直起跳控制器

jump_controller.py 把起跳分成四个阶段：



基本力规划：



检查阶段输出：



检查时间序列：



画普通力矩曲线：



输出：



画平滑力矩曲线：



输出：



## 10. smoothstep

用于把力从一个值平滑过渡到另一个值：



特点：



用途：减少力矩突变，避免对电机和结构造成冲击。

## 11. MuJoCo 加载 URDF

检查 MuJoCo 是否能加载 URDF：



曾经遇到的问题：



原因：URDF 里有零尺寸 collision：



修复脚本：



修复后备份文件：



MuJoCo 当前输出：



含义：



下一步要做：



## 12. 原仓库控制逻辑

原仓库不是直接规划关节角。

它的控制链路是：



核心思想：



## 13. 常用命令

进入项目：



查看文件：

./.git/FETCH_HEAD
./.git/HEAD
./.git/config
./.git/description
./.git/hooks/applypatch-msg.sample
./.git/hooks/commit-msg.sample
./.git/hooks/fsmonitor-watchman.sample
./.git/hooks/post-update.sample
./.git/hooks/pre-applypatch.sample
./.git/hooks/pre-commit.sample
./.git/hooks/pre-merge-commit.sample
./.git/hooks/pre-push.sample
./.git/hooks/pre-rebase.sample
./.git/hooks/pre-receive.sample
./.git/hooks/prepare-commit-msg.sample
./.git/hooks/push-to-checkout.sample
./.git/hooks/sendemail-validate.sample
./.git/hooks/update.sample
./.git/index
./.git/info/exclude
./.git/logs/HEAD
./.git/packed-refs
./.gitignore
./.vscode/settings.json
./0
./12
./3
./Centroidal
./LICENSE
./MuJoCo
./README.md
./convex_mpc_2fix.pdf
./environment.yml
./examples/__init__.py
./examples/__pycache__/__init__.cpython-310.pyc
./examples/__pycache__/ex00_demo.cpython-310.pyc
./examples/ex00_demo.py
./examples/ex01_trot_in_place.py
./examples/ex02_trot_forward.py
./examples/ex03_trot_sideway.py
./examples/ex04_trot_rotation.py
./media/forward_walking.gif
./media/mpc_state_force_logs.png
./media/mpc_timing_stats.png
./media/side_walking.gif
./media/yaw_rotation.gif
./models/URDF/local_mqqpf0fp_c38c2r_urdf_stl .zip
./pyproject.toml
./src/convex_mpc/__init__.py
./src/convex_mpc/centroidal_mpc.py
./src/convex_mpc/com_trajectory.py
./src/convex_mpc/gait.py
./src/convex_mpc/go2_robot_data.py
./src/convex_mpc/leg_controller.py
./src/convex_mpc/mujoco_model.py
./src/convex_mpc/plot_helper.py
./stance
./swing
./tests/__pycache__/test_custom_robot_data.cpython-310.pyc
./生成

运行模型检查：



运行站立控制器：



运行起跳时间线：



画平滑起跳力矩图：



检查 MuJoCo 加载：



## 14. 当前进度总结

已经完成：



下一步：


