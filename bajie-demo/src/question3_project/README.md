# 题三 · Isaac Sim 玩具触碰

在官方场景 `scene/abr_scene.usd` 中逐例复位机器人底盘、关节和目标玩具，复用题二的官方 URDF 正运动学规划夹爪的接近与下降轨迹，通过 ROS 2 Bridge 下发关节指令。只有 PhysX 确认**目标玩具**与**夹爪左指或右指**发生真实接触时，才读取该物理步的实际关节状态和相对位姿并记为作答。场景与模型只读挂载，约束和 ROS 图只写入内存会话层。

工作区已在 Isaac Sim 4.5 / ROS 2 Humble 中用 `adaptive` 和下述严格判据复测三例，均为左指和对应玩具接触，无回退。仓库里的[原三行作答](../../answers/q3_demo_output.jsonl)和[原运行日志](../../assets/q3_run.log)保留作历史记录，使用的是修正前允许正间隙的判据，不是这次复测的证明。

## 目录结构

| 路径                                          | 作用                                                                                               |
| --------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| [`run_touch.py`](run_touch.py)               | 主程序：启动 Kit、打开场景、复位与约束、规划、下发指令、订阅接触与关节反馈、写作答                 |
| [`touch.py`](touch.py)                       | 纯逻辑：读取三例输入、`Planner` 增量 IK、smoothstep 采样、接触判定、作答组装、原子写文件         |
| [`bridge.py`](bridge.py)                     | 在内存会话层创建 ROS 2 Action Graph：时钟、关节状态发布、关节指令订阅与 `ArticulationController` |
| [`run_laptop.sh`](run_laptop.sh)             | 笔记本侧 Docker 启动器：资源/显存预检、只读挂载、资源限制、watchdog、GUI 与缓存挂载                |
| [`physics.kit`](physics.kit)                 | 无界面体验配置：只用 USD/时间线/PhysX/ROS 2 Bridge，关闭渲染与窗口                                 |
| [`bridge_probe.py`](bridge_probe.py)         | 旧桥接诊断：只验证 `/clock` 与 `/joint_states`，**不用于正式复现**                       |
| [`tests/test_touch.py`](tests/test_touch.py) | 输入契约、坐标系换算、smoothstep、接触配对                                                         |

## 环境与依赖

- 官方仿真资源 [https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim)。本工作区已完整放在 `src/bajie_sim`（含 `scene/`、`machine/`、`models/`），无需再下载；换机器时必须保留完整的 `models/` 相对目录，只下载入口文件不可用。
- 按[题二说明](../question2_project/README.md)准备的官方 URDF，本工作区已展开为 `generated/bajie_robot.urdf`，供本题规划求解使用。
- Docker、可用的 NVIDIA 容器运行时与 `nvidia-smi`；镜像基于官方 `nvcr.io/nvidia/isaac-sim:4.5.0` 并安装 ROS 2 Humble（Python 3.10，使用 Isaac Sim 自带的 NumPy 与 SciPy）。本机镜像标签为 `bajie/sim:humble45`，可用 `Q3_IMAGE` 指定同环境的其他标签；完整开发工作区提供 `docker/Dockerfile.sim` 作为构建参考。

## 运行

在**作品根目录 `bajie-demo/`**执行：

```bash
export BAJIE_SIM_DIR=/home/moke/bajie_ws/src/bajie_sim
export BAJIE_URDF=/home/moke/bajie_ws/generated/bajie_robot.urdf
set -o pipefail
out=$(mktemp -d /tmp/bajie-q3-verified.XXXXXX)
Q3_STRATEGY=adaptive Q3_RESULT_DIR="$out" \
  bash src/question3_project/run_laptop.sh 2>&1 | tee "$out/run.log"
```

本机工作区路径为 `/home/moke/bajie_ws/src/bajie_sim` 与 `/home/moke/bajie_ws/generated/bajie_robot.urdf`；资源迁入工作区后已用这两个路径复跑三例并通过。

### 启动器参数（环境变量）

| 变量              | 默认                      | 说明                                                                                          |
| ----------------- | ------------------------- | --------------------------------------------------------------------------------------------- |
| `BAJIE_SIM_DIR` | 必填                      | 含 `scene/abr_scene.usd` 与 `models/` 的官方资源根目录                                    |
| `BAJIE_URDF`    | 必填                      | 展开后的官方 URDF 绝对路径                                                                    |
| `Q3_RESULT_DIR` | `bajie-demo/answers`    | 输出目录，必须已存在；容器内挂到 `/result`                                                  |
| `Q3_STRATEGY`   | `baseline`              | `adaptive` 只试轻触新动作；`adaptive-fallback` 遇到预期接触失败时，重新加载场景并回退该例 |
| `Q3_IMAGE`      | `bajie/sim:humble45`    | 容器镜像标签                                                                                  |
| `Q3_GUI`        | `0`                     | `1` 时打开 Isaac Sim 原生窗口；需要 `DISPLAY`，改用 host 网络与 12 GiB 内存               |
| `Q3_CACHE_DIR`  | `~/.cache/bajie-q3-kit` | GUI 模式下的 Kit 着色器/数据缓存，需持久化，否则每次重编 RTX 着色器                           |

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
4. 接触判定同时满足：接触双方必须是指定玩具与 `gripper_left_link`/`gripper_right_link`；接触冲量模 > `1e-6`；**接触分离距离 ≤ `0 m`**。PhysX 在表面仍有正间隙时也可能报告带冲量的事件，此时继续原定低速轨迹，不能算成功。基线首次确认接触后尝试再下压最多 3 mm，保留最后一次有效接触的快照；自适应策略在触碰阶段首次确认接触就结束接近动作，不再收拢或下压。两种策略均从有效接触的同一步读取 10 个实际关节及三个坐标系的变换。
5. 保存接触快照后，保持当前仿真场景，沿已走过的实际关节姿态逆序发送限速指令，先缓慢退出玩具，再收回机械臂；不直接设置归位姿态。只有全部 10 个关节连续 12 帧满足位置误差（升降 ≤ 2 mm，其余 ≤ 0.01 rad）、速度（升降 ≤ 0.02 m/s，其余 ≤ 0.02 rad/s）且不再接触目标，才记为 `return_home=settled` 并切换到下一独立测例。归位中发现非目标碰撞、脱离后重新接触或停稳超时则中止，既不切下一例，也不触发回退。
6. 三例都完成触碰并逐步归位后，才按输入顺序**原子替换** `q3_demo_output.jsonl`。归位不会修改已经保存的触碰姿态。`--case` 同样执行归位，只打印单例结果，不写正式作答。

### 作答字段

- `final_joint_state`：10 个关节的实际位置。
- `objects_link_in_base_footprint`：夹爪相对底盘，`xyz` 米、`rpy` 弧度、`quat_xyzw` 顺序 `[x, y, z, w]`。
- `scene_object_in_objects_link`：玩具相对夹爪。

输入见 [`q3_demo_input.jsonl`](../../inputs/q3_demo_input.jsonl)，每例给出 `base_footprint_in_world`、`scene_object_in_world`、初始 `joint_state` 与玩具名（`rubik_cube` / `dinosaur` / `soft_toy`）。

### 自适应轻触与对照

- **魔方**：让闭合方向对齐实际玩具坐标系的一条侧面法向；在高处转腕和张开，下降到侧面上部，再固定手腕低速收指。
- **恐龙**：官方场景试验中，两指从窄侧收拢容易跨过收窄的轮廓而不接触。优先找主体上部作单指轻触，避免硬包头尾；没有可达的上部候选时才尝试窄侧候选。
- **软玩具**：优先用一侧指垫对准上部的宽表面，最后短距离下降并小幅收指；不强求双指接触。

规划使用官方 USD 启用的网格/方盒碰撞几何和官方 URDF 指垫盒；这是保守筛选，不是完整的全臂避碰证明。若提前碰到玩具、监测到非目标碰撞或到达末点仍无真实接触，不计为新动作成功。`adaptive-fallback` 此时关闭并重新打开官方场景，恢复输入里的底盘、玩具、关节后再走基线；桥接/场景错误不会被当成接触失败吞掉。

对照试验使用单独目录，避免覆盖提交答案和已有日志。新策略的三例输出要求显式指定非正式答案目录：

```bash
set -o pipefail
compare=$(mktemp -d /tmp/bajie-q3-compare.XXXXXX)
mkdir -p "$compare/baseline" "$compare/adaptive"
Q3_STRATEGY=baseline Q3_RESULT_DIR="$compare/baseline" \
  bash src/question3_project/run_laptop.sh 2>&1 | tee "$compare/baseline/run.log"
Q3_STRATEGY=adaptive-fallback Q3_RESULT_DIR="$compare/adaptive" \
  bash src/question3_project/run_laptop.sh 2>&1 | tee "$compare/adaptive/run.log"
grep -h 'outcome=' "$compare"/*/run.log
```

每条 `outcome=` 单独报告策略、成败、阶段、指令步数、目标原点最大平移（毫米）、非目标接触步数和未被接受的最小正间隙。比较时逐例看 `strategy=adaptive` 是否真正成功，不把回退结果计入新动作成功率。题面没有公开这几个指标的计分权重。

#### 严格接触判据复测（2026-11-21）

Isaac Sim 4.5 / ROS 2 Humble，无界面，`adaptive`。三个测例均在触碰阶段获得带冲量的非正分离距离，没有回退、没有监测到非目标接触。位移为单轮执行中玩具原点的最大平移。

| 玩具   | 动作                         | 接触分离距离 | 玩具最大平移 |
| ------ | ---------------------------- | -----------: | -----------: |
| 魔方   | 固定手腕，收拢时左指碰侧面   |  −0.0075 mm |     0.041 mm |
| 恐龙   | 左指向主体上部短距下降并轻合 |  −0.0810 mm |     0.204 mm |
| 软玩具 | 左指轻触上部，触碰就停       |  −0.0216 mm |     0.056 mm |

复测文件单独保存在工作区的 `logs/q3-surface-headless.q4R5iP/`。此前的 `logs/q3-compare-2026-11-21/` 使用允许正间隙的旧判据，两轮结果不能直接对比。推荐用本次验证过的 `adaptive` 复现；正式答案及提交日志未自动替换。

## 原生 GUI 仿真

默认以无界面方式运行；需要观察三维画面时使用 Isaac Sim 原生窗口：

```bash
out=$(mktemp -d /tmp/bajie-q3-gui.XXXXXX)
Q3_GUI=1 Q3_STRATEGY=adaptive Q3_RESULT_DIR="$out" \
  bash src/question3_project/run_laptop.sh 2>&1 | tee "$out/run.log"
```

启动器会挂载 X11 与 Kit 缓存（`Q3_CACHE_DIR`），窗口分辨率 960×540，相机先对准机器人。获取作答快照后暂停物理，展示接触位置约 2.5 秒，然后切回全景并继续仿真，展示退出和归位动作。GUI 模式内存上限 12 GiB、运行上限 900 秒，需要可用的 `DISPLAY`；启动前必须满足可用内存 12,800 MiB。首次启动需编译 RTX 着色器，耗时较长。

## 测试

不启动仿真，只需 Python 3.10+、NumPy 与 SciPy：

```bash
python3 -m unittest discover -s src/question3_project/tests -v
```

覆盖三例输入与作答坐标系、轨迹端点与速度限制、按形状转腕、用官方 URDF 求解三例动作、夹爪几何筛选、首触即停、失败回退、逆序归位和停稳门槛。自适应规划测试使用合成碰撞点；测试需要工作区的 `generated/bajie_robot.urdf`，独立提交目录可用 `BAJIE_URDF` 指定。真实接触和归位仍须用 Isaac Sim 验证。

## 常见问题

| 现象                                                        | 原因与处理                                                                                  |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `Set BAJIE_SIM_DIR ...` / `Missing output directory...` | 环境变量未设、`Q3_RESULT_DIR` 不存在，或场景缺少 `models/`                              |
| `未启动：系统可用内存不足 12800 MiB`                      | 先关闭浏览器/IDE/其他仿真；这是启动前的硬门槛                                               |
| `未启动：显存余量不足 6000 MiB`                           | 释放显存，确认 `nvidia-smi` 可用且 NVIDIA 容器运行时已配置                                |
| `Missing official scene prim` / `Missing toy colliders` | 官方资源不完整或目录层级不对，重新下载完整 `models/`                                      |
| `ROS handshake failed`                                    | Bridge 未启用或 ROS 域冲突；确认镜像内含 `isaacsim.ros2.bridge`，并避免与宿主其他演示同域 |
| `Official FK does not match sim`                          | URDF 与场景模型不是同一版本，或底盘/玩具摆位错误                                            |
| `no verified fingertip contact`                           | 规划路径未真正触碰；先确认玩具位置与 `models/` 完整，再单独用 `--case` 排查             |
| `return_home=failed`                                      | 归位遇碰撞、重碰目标或超时未停稳；先检查该例日志，程序不会跳到下一例                        |
| 容器被 watchdog 终止                                        | 内存/显存触线或超时；关闭其他程序，或临时降低并发需求后重试                                 |
| `Q3_GUI=1` 但无窗口                                       | `DISPLAY` 为空、Xauthority 不可用，或渲染缓存目录不可写                                   |

## 注意事项

- 本题依赖物理接触，**不能用题一的几何预检结果代替**；作答中的姿态是仿真实际读数。
- 不要改写官方 USD；脚本只使用会话层，若需要保存调试图请另存副本。
- GUI 模式会显著增加内存与显存占用，正式跑三例建议保持无界面模式。
- 必交演示视频预计命名 `q3_touch.mp4`，当前尚未生成；视频与作答是两项独立材料。
