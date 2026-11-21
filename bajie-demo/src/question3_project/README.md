# 题三 · Isaac Sim 玩具触碰

在官方场景 `scene/abr_scene.usd` 中逐例复位机器人底盘、关节和目标玩具，复用题二的官方 URDF 正运动学规划夹爪的接近与下降轨迹，通过 ROS 2 Bridge 下发关节指令。只有 PhysX 确认**目标玩具**与**夹爪左指或右指**发生真实接触时，才读取该物理步的实际关节状态和相对位姿并记为作答。场景与模型只读挂载，约束和 ROS 图只写入内存会话层。

三个正式测例均已在 Isaac Sim 4.5 / ROS 2 Humble 上实测：[三行作答](../../answers/q3_demo_output.jsonl)、[完整运行日志](../../assets/q3_run.log)。接触记录：`touch_toy_1` 魔方由**右指**触碰，`touch_toy_2` 恐龙由**左指**触碰，`touch_toy_3` 软玩具由**右指**触碰。

## 目录结构

| 路径 | 作用 |
| --- | --- |
| [`run_touch.py`](run_touch.py) | 主程序：启动 Kit、打开场景、复位与约束、规划、下发指令、订阅接触与关节反馈、写作答 |
| [`touch.py`](touch.py) | 纯逻辑：读取三例输入、`Planner` 增量 IK、smoothstep 采样、接触判定、作答组装、原子写文件 |
| [`bridge.py`](bridge.py) | 在内存会话层创建 ROS 2 Action Graph：时钟、关节状态发布、关节指令订阅与 `ArticulationController` |
| [`run_laptop.sh`](run_laptop.sh) | 笔记本侧 Docker 启动器：资源/显存预检、只读挂载、资源限制、watchdog、GUI 与缓存挂载 |
| [`physics.kit`](physics.kit) | 无界面体验配置：只用 USD/时间线/PhysX/ROS 2 Bridge，关闭渲染与窗口 |
| [`bridge_probe.py`](bridge_probe.py) | 旧桥接诊断：只验证 `/clock` 与 `/joint_states`，**不用于正式复现** |
| [`tests/test_touch.py`](tests/test_touch.py) | 输入契约、坐标系换算、smoothstep、接触配对 |

## 环境与依赖

- 官方仿真资源 <https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim>。本工作区已完整放在 `src/bajie_sim`（含 `scene/`、`machine/`、`models/`），无需再下载；换机器时必须保留完整的 `models/` 相对目录，只下载入口文件不可用。
- 按[题二说明](../question2_project/README.md)准备的官方 URDF，本工作区已展开为 `generated/bajie_robot.urdf`，供本题规划求解使用。
- Docker、可用的 NVIDIA 容器运行时与 `nvidia-smi`；镜像基于官方 `nvcr.io/nvidia/isaac-sim:4.5.0` 并安装 ROS 2 Humble（Python 3.10，使用 Isaac Sim 自带的 NumPy 与 SciPy）。本机镜像标签为 `bajie/sim:humble45`，可用 `Q3_IMAGE` 指定同环境的其他标签；完整开发工作区提供 `docker/Dockerfile.sim` 作为构建参考。

## 运行

在**作品根目录 `bajie-demo/`**执行：

```bash
export BAJIE_SIM_DIR=/home/moke/bajie_ws/src/bajie_sim
export BAJIE_URDF=/home/moke/bajie_ws/generated/bajie_robot.urdf
set -o pipefail
bash src/question3_project/run_laptop.sh 2>&1 | tee assets/q3_run.log
```

本机工作区路径为 `/home/moke/bajie_ws/src/bajie_sim` 与 `/home/moke/bajie_ws/generated/bajie_robot.urdf`；资源迁入工作区后已用这两个路径复跑三例并通过。

### 启动器参数（环境变量）

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `BAJIE_SIM_DIR` | 必填 | 含 `scene/abr_scene.usd` 与 `models/` 的官方资源根目录 |
| `BAJIE_URDF` | 必填 | 展开后的官方 URDF 绝对路径 |
| `Q3_RESULT_DIR` | `bajie-demo/answers` | 输出目录，必须已存在；容器内挂到 `/result` |
| `Q3_IMAGE` | `bajie/sim:humble45` | 容器镜像标签 |
| `Q3_GUI` | `0` | `1` 时打开 Isaac Sim 原生窗口；需要 `DISPLAY`，改用 host 网络与 12 GiB 内存 |
| `Q3_CACHE_DIR` | `~/.cache/bajie-q3-kit` | GUI 模式下的 Kit 着色器/数据缓存，需持久化，否则每次重编 RTX 着色器 |

单独排查一例（**不会**写正式作答）：

```bash
bash src/question3_project/run_laptop.sh --case rubik_cube
# 可选：dinosaur、soft_toy
```

### 资源保护与自动终止

- 启动前要求宿主可用内存 ≥ **12,800 MiB**、空闲显存 ≥ **6,000 MiB**，否则直接不启动。
- 无界面容器限制 **8 GiB 内存、禁止交换、4 CPU、768 PID**，网络隔离；GUI 模式为 **12 GiB、host 网络**。
- watchdog 每 2 秒采样：可用内存 < **4,500 MiB** 或空闲显存 < **500 MiB**，连续两次触线即停止容器；运行满 **360 秒**（GUI 为 900 秒）同样终止。请勿同时启动 RViz 或其他仿真实例。
- 脚本只停止自己创建的容器（固定名 `bajie-q3-safe`），并在退出时清理。

正式实测最低可用内存 **10,037 MiB**、最低空闲显存 **6,324 MiB**（采样值）。

## 运行逻辑

1. `run_touch.py` 逐例重新打开官方场景，将所有 `OmniGraph` 置为 inactive，只保留本题自己创建的桥接图；把底盘与玩具按输入位姿摆到世界中，并用内存中的 `FixedJoint` 锚定底盘。
2. 设置 CPU PhysX、60 Hz 固定步长，并在玩具与两个指尖上启用接触报告；`touch.py` 从官方 URDF 的 `gripper_left_link` 碰撞盒算出指尖垫相对 `objects_link` 的偏移（约向下 8.5 mm、侧向 40 mm），用**指尖垫**而不是工具原点对准玩具，再以 4 mm 步长下降；下降深度取玩具包围盒顶部与放置原点中的较低者，避免不规则玩具（如恐龙的最高点在头部）只被“够到上方空气”。
3. `bridge.py` 建立 `/q3/joint_states`（发布）、`/q3/joint_command`（订阅）、`/q3/clock`，由 `ArticulationController` 驱动 articulation。只有指 1 接收位置指令，指 2 是 USD 侧 mimic。
4. 接触判定同时满足：接触双方必须是指定玩具与 `gripper_left_link`/`gripper_right_link`；接触冲量模 > `1e-6`；接触分离距离 ≤ `0.001 m`。首次接触后继续下压最多 3 mm（每毫米一步），只有接触持续存在才读取该步的 10 个实际关节、`base_footprint`/`objects_link`/玩具的世界位姿并组装作答。
5. 三例全部通过后，才按输入顺序**原子替换** `q3_demo_output.jsonl`；任何一例失败都不会写出正式作答。`--case` 只校验单例并打印结果。

### 作答字段

- `final_joint_state`：10 个关节的实际位置。
- `objects_link_in_base_footprint`：夹爪相对底盘，`xyz` 米、`rpy` 弧度、`quat_xyzw` 顺序 `[x, y, z, w]`。
- `scene_object_in_objects_link`：玩具相对夹爪。

输入见 [`q3_demo_input.jsonl`](../../inputs/q3_demo_input.jsonl)，每例给出 `base_footprint_in_world`、`scene_object_in_world`、初始 `joint_state` 与玩具名（`rubik_cube` / `dinosaur` / `soft_toy`）。

## 原生 GUI 仿真

默认以无界面方式运行；需要观察三维画面时使用 Isaac Sim 原生窗口：

```bash
Q3_GUI=1 bash src/question3_project/run_laptop.sh
```

启动器会挂载 X11 与 Kit 缓存（`Q3_CACHE_DIR`），窗口分辨率 960×540，相机自动对准机器人。GUI 模式内存上限 12 GiB、运行上限 900 秒，需要可用的 `DISPLAY`。首次启动需编译 RTX 着色器，耗时较长。

## 测试

不启动仿真，只需 Python 3.10+、NumPy 与 SciPy：

```bash
python3 -m unittest discover -s src/question3_project/tests -v
```

覆盖：三例输入与目标坐标、作答坐标系换算、smoothstep 端点与速度上限、接触必须是指定玩具与指定手指。

## 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| `Set BAJIE_SIM_DIR ...` / `Missing output directory...` | 环境变量未设、`Q3_RESULT_DIR` 不存在，或场景缺少 `models/` |
| `未启动：系统可用内存不足 12800 MiB` | 先关闭浏览器/IDE/其他仿真；这是启动前的硬门槛 |
| `未启动：显存余量不足 6000 MiB` | 释放显存，确认 `nvidia-smi` 可用且 NVIDIA 容器运行时已配置 |
| `Missing official scene prim` / `Missing toy colliders` | 官方资源不完整或目录层级不对，重新下载完整 `models/` |
| `ROS handshake failed` | Bridge 未启用或 ROS 域冲突；确认镜像内含 `isaacsim.ros2.bridge`，并避免与宿主其他演示同域 |
| `Official FK does not match sim` | URDF 与场景模型不是同一版本，或底盘/玩具摆位错误 |
| `no verified fingertip contact` | 规划路径未真正触碰；先确认玩具位置与 `models/` 完整，再单独用 `--case` 排查 |
| 容器被 watchdog 终止 | 内存/显存触线或超时；关闭其他程序，或临时降低并发需求后重试 |
| `Q3_GUI=1` 但无窗口 | `DISPLAY` 为空、Xauthority 不可用，或渲染缓存目录不可写 |

## 注意事项

- 本题依赖物理接触，**不能用题一的几何预检结果代替**；作答中的姿态是仿真实际读数。
- 不要改写官方 USD；脚本只使用会话层，若需要保存调试图请另存副本。
- GUI 模式会显著增加内存与显存占用，正式跑三例建议保持无界面模式。
- 必交演示视频预计命名 `q3_touch.mp4`，当前尚未生成；视频与作答是两项独立材料。
