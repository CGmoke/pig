# 八界机器人：关节展示、运动学与仿真触碰

参赛者 / 团队：**待填写报名信息**。本目录为初赛作品整理区，尚未达到正式提交状态。

| 题目 | 源码 | 正式输入 | 当前状态 |
| --- | --- | --- | --- |
| 题一 · 关节分步驱动 | [`src/question1_project/`](src/question1_project/) | [`q1_demo_input.jsonl`](inputs/q1_demo_input.jsonl) | 已用 Humble 无界面运行生成[作答](answers/q1_demo_output.jsonl)；待用 RViz 复跑并录制视频 |
| 题二 · 正向运动学 | [`src/question2_project/`](src/question2_project/) | [`q2_demo_input.jsonl`](inputs/q2_demo_input.jsonl) | 已生成[作答](answers/q2_demo_output.jsonl)；8 项自动测试及 Humble TF 三组交叉复核通过 |
| 题三 · 仿真触碰 | [`src/question3_project/`](src/question3_project/) | [`q3_demo_input.jsonl`](inputs/q3_demo_input.jsonl) | 三例均已实测触碰并生成[作答](answers/q3_demo_output.jsonl)和[日志](assets/q3_run.log)；演示视频待补 |

## 环境与官方资源

- 题一、题二基于 **Ubuntu 22.04 / ROS 2 Humble** 和[官方 URDF 仓库](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf)。模型入口是 `urdf/bajie_robot.urdf.xacro`，请保留原有几何和关节 `origin`。
- 题三使用 **Isaac Sim 4.5 / ROS 2 Bridge** 和[官方仿真资源仓库](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim)，入口是 `scene/abr_scene.usd`；保留场景依赖的相对目录，不要只下载入口文件。
- 资源不随作品重复提交。本工作区已备好完整资源：URDF 在 `../src/bajie_robot`，仿真场景在 `../src/bajie_sim`（含 `scene/`、`machine/`、`models/`），无需再下载。换机器时再用 `BAJIE_URDF_DIR`、`BAJIE_SIM_DIR` 指向各自的绝对路径；仿真资源必须包含 `models/`，只有场景入口的副本不可用。[依赖识别与提交自查](docs/提交自查.md)列出了当前模型入口的校验值。

## 题一：构建并运行

以下命令从**本 README 所在目录**执行。需要已经安装 ROS 2 Humble、`colcon`，以及 `rclpy`、`sensor_msgs`、`xacro`、`python3-numpy`、`robot_state_publisher`、`rviz2`、`ament_cmake` 等依赖。RViz 需要可用图形桌面。

```bash
export BAJIE_URDF_DIR="$(cd .. && pwd)/src/bajie_robot"
source /opt/ros/humble/setup.bash   # 宿主只有 Jazzy 时改用 /opt/ros/jazzy/setup.bash
colcon build --base-paths src/question1_project "$BAJIE_URDF_DIR" \
  --packages-select bajie_robot bajie_tasks --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

set -o pipefail
ros2 launch bajie_tasks q1_rviz.launch.py \
  input_file:="$PWD/inputs/q1_demo_input.jsonl" \
  output_file:="$PWD/answers/q1_demo_output.jsonl" 2>&1 | tee assets/q1_run.log
```

观察升降 → 头部 → 机械臂 → 夹爪张开 → 夹爪闭合。作答文件应有一行，包含五个按序排列的关键帧及最终状态。动作完成并写出答案后，可关闭 RViz 或结束启动程序。

**轨迹避碰：**机械臂朝前上方抬展，仍由 `seed` 决定目标。程序根据官方 URDF 的可视及碰撞几何构造包围盒，正向计算各连杆位姿，用区间内最大位移给净空作保守估计。每段轨迹（含夹爪开闭、多例之间的回零）需保守满足离 `base_footprint` 下的 `z=0` 地面至少 **5 cm**、离非相连机身至少 **2 cm** 才会播放。机械臂第一节与安装座的固有连接界面不计入；这是题一 RViz 模型的几何验证，不是物理接触仿真。检查不通过时停止，不写该例答案。

验证轨迹停播条件和种子 `0–127`（无需启动 RViz，先构建并加载环境）：

```bash
python3 -m unittest discover -s src/question1_project/test -v
```

## 题二：正向运动学与坐标变换

从**本 README 所在目录**执行。工作区已展开好 `../generated/bajie_robot.urdf`，直接设 `BAJIE_URDF` 即可；换机器时按[题二说明](src/question2_project/README.md)重新展开。求解只需要 Python 3.10+ 和 NumPy，不需要启动 RViz、Isaac Sim 或 ROS 节点。

```bash
export BAJIE_URDF="$(cd .. && pwd)/generated/bajie_robot.urdf"
set -o pipefail
python3 src/question2_project/solve_q2.py --urdf "$BAJIE_URDF" \
  --input inputs/q2_demo_input.jsonl --output answers/q2_demo_output.jsonl 2>&1 | tee assets/q2_run.log
```

程序从官方 URDF 提取底盘到夹爪的关节链，不硬编码题面答案。独立计算出的夹爪位姿和物体相对夹爪的位姿，均由 Humble TF2 复核；见[复核日志](assets/q2_humble_verify.log)。[题二说明](src/question2_project/README.md)提供测试命令和“只升降 0.1 m”的手算练习。

## 题三：Isaac Sim 玩具触碰

魔方、恐龙、软玩具独立复位后，程序使用官方 URDF 规划夹爪的接近和下降，通过 ROS 2 Bridge 驱动机械臂。只有 PhysX 确认目标玩具与夹爪手指真正接触，才读取该时刻的关节状态和相对位姿。使用 CPU 物理、无渲染运行，不改写官方场景文件。

在**本 README 所在目录**执行。官方场景与 URDF 已在本工作区，另需 Isaac Sim 4.5 / ROS 2 Humble 镜像（细节见[题三运行说明](src/question3_project/README.md)）：

```bash
export BAJIE_SIM_DIR="$(cd .. && pwd)/src/bajie_sim"
export BAJIE_URDF="$(cd .. && pwd)/generated/bajie_robot.urdf"
set -o pipefail
bash src/question3_project/run_laptop.sh 2>&1 | tee assets/q3_run.log
```

输入：[三组测例](inputs/q3_demo_input.jsonl)。输出：[正式作答](answers/q3_demo_output.jsonl)和[完整日志](assets/q3_run.log)。三例均通过才替换答案。题三必交视频预计命名为 `q3_touch.mp4`，尚未生成。

## 提交材料

- [`assets/`](assets/README.md)：提交用的完整运行日志。题三实测见 `q3_run.log`，原桥接基线仅供诊断。
- [`video/`](video/README.md)：题一 RViz 和题三 Isaac Sim 视频均为必交；单个文件不超过 **30,000,000 字节**。视频完成后在此 README 填写实际文件名。
- [`docs/提交自查.md`](docs/提交自查.md)：正式提交前核对清单。请填写参赛信息和实际完成的亮点，将本目录改名为报名姓名后放入独立 AtomGit 作品仓库，不要上传外层开发工作区。
