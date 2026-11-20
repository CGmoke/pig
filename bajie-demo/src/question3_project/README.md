# 题三 · Isaac Sim 玩具触碰

三个正式测例均已在 Isaac Sim 4.5 / ROS 2 Humble 上实测：[三行作答](../../answers/q3_demo_output.jsonl)、[完整运行日志](../../assets/q3_run.log)。`touch_toy_1` 魔方由右指触碰，`touch_toy_2` 恐龙由左指触碰，`touch_toy_3` 软玩具由右指触碰。该日志记录了三次接触及容器正常退出；演示视频状态见[视频清单](../../video/README.md)。

## 笔记本复现

需要 Docker、可用的 NVIDIA 容器运行时、`nvidia-smi`，以及基于官方 `nvcr.io/nvidia/isaac-sim:4.5.0`、安装 ROS 2 Humble 的容器镜像（Python 3.10，使用 Isaac Sim 中的 NumPy 和 SciPy）。本机镜像标签为 `bajie/sim:humble45`，可用 `Q3_IMAGE` 指定同环境的其他标签。完整开发工作区提供 `docker/Dockerfile.sim` 作为镜像构建参考。

1. 按[作品说明](../../README.md)取得官方仿真资源，保留 `scene/abr_scene.usd` 和完整 `models/`，以及场景依赖的相对目录。
2. 按[题二说明](../question2_project/README.md)把官方 Xacro 展开为 URDF，供本题规划求解使用。
3. 在**作品根目录**运行：

```bash
export BAJIE_SIM_DIR=/绝对路径/完整官方场景目录
export BAJIE_URDF=/绝对路径/bajie_robot.urdf
set -o pipefail
bash src/question3_project/run_laptop.sh 2>&1 | tee assets/q3_run.log
```

本次笔记本实测路径分别为 `/home/moke/WorkBuddy/Eight-Realms` 和 `/home/moke/bajie_ws/generated/bajie_robot.urdf`。可加 `--case rubik_cube`、`--case dinosaur` 或 `--case soft_toy` 单独排查；单例不会覆盖正式答案。也可用 `Q3_RESULT_DIR` 指向一个已存在的目录，单独保留复跑输出。

启动脚本要求宿主可用内存至少 **12,800 MiB**、空闲显存至少 **6,000 MiB**；容器限制为 **8 GiB 内存、禁止容器交换、4 个 CPU 的配额**，每 2 秒检查一次：可用内存低于 **4,500 MiB** 或空闲显存低于 **500 MiB**，连续两次检查触线就终止容器；运行满 **360 秒** 也会终止。请勿同时启动 RViz 或其他仿真实例。正式实测监控最低可用内存为 **10,037 MiB**，最低空闲显存为 **6,324 MiB**（采样值）。

## 运行逻辑

- [`run_touch.py`](run_touch.py) 逐例重开官方场景，将底盘、指定玩具和关节恢复到给定状态；底盘约束、CPU PhysX 和任务 ROS 图仅写入内存会话层。模型以只读方式挂载，不改官方 USD。
- [`touch.py`](touch.py) 将世界坐标下的玩具变换到底盘系，复用题二的官方 URDF 正向运动学，在 URDF 关节限位内求解接近、下降轨迹。 [`bridge.py`](bridge.py) 使用 ROS 2 Bridge 发送关节指令，并检查时钟和关节反馈。
- 只接受指定玩具与夹爪左指或右指的 PhysX 接触：接触冲量模大于 `1e-6`、接触分离距离不超过 `0.001 m`。通过即读取当步的实际十个关节、夹爪相对底盘位姿和玩具相对夹爪位姿。位移用米、RPY 用弧度，四元数顺序为 `[x, y, z, w]`。
- 输入为 [`q3_demo_input.jsonl`](../../inputs/q3_demo_input.jsonl)，仅在三例都通过后按输入顺序原子替换 `answers/q3_demo_output.jsonl`。 [`bridge_probe.py`](bridge_probe.py) 是旧桥接诊断，不用于正式复现。

不启动仿真的单元测试（需 Python 3.10+、NumPy 和 SciPy）：

```bash
python3 -m unittest discover -s src/question3_project/tests -v
```
