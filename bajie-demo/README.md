# 八界机器人：关节展示、运动学与仿真触碰

参赛者 / 团队：**待填写报名信息**。本目录为初赛作品整理区，尚未达到正式提交状态。

| 题目 | 源码 | 正式输入 | 当前状态 |
| --- | --- | --- | --- |
| 题一 · 关节分步驱动 | [`src/question1_project/`](src/question1_project/) | [`q1_demo_input.jsonl`](inputs/q1_demo_input.jsonl) | 已用 Humble 无界面运行生成[作答](answers/q1_demo_output.jsonl)；待用 RViz 复跑并录制视频 |
| 题二 · 正向运动学 | [`src/question2_project/`](src/question2_project/) | [`q2_demo_input.jsonl`](inputs/q2_demo_input.jsonl) | 待实现；无正式作答 |
| 题三 · 仿真触碰 | [`src/question3_project/`](src/question3_project/) | [`q3_demo_input.jsonl`](inputs/q3_demo_input.jsonl) | 已有 ROS 2 桥接探针；待完成三例触碰控制、作答及视频 |

## 环境与官方资源

- 题一、题二基于 **Ubuntu 22.04 / ROS 2 Humble** 和[官方 URDF 仓库](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf)。模型入口是 `urdf/bajie_robot.urdf.xacro`，请保留原有几何和关节 `origin`。
- 题三使用 **Isaac Sim 4.5 / ROS 2 Bridge** 和[官方仿真资源仓库](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim)，入口是 `scene/abr_scene.usd`；保留场景依赖的相对目录，不要只下载入口文件。
- 资源不随作品重复提交。下载官方完整资源后，下面用 `BAJIE_URDF_DIR`、`BAJIE_SIM_DIR` 指向各自的绝对路径。当前开发工作区已有这两套资源，可直接指向其中的 `../src/bajie_robot` 和 `../src/bajie_sim`（相对于本作品目录）。[依赖识别与提交自查](docs/提交自查.md)列出了当前模型入口的校验值。

## 题一：构建并运行

以下命令从**本 README 所在目录**执行。需要已经安装 ROS 2 Humble、`colcon`，以及 `rclpy`、`sensor_msgs`、`xacro`、`python3-numpy`、`robot_state_publisher`、`rviz2`、`ament_cmake` 等依赖。RViz 需要可用图形桌面。

```bash
export BAJIE_URDF_DIR=/你的模型目录/ecovacs-bajie-urdf
source /opt/ros/humble/setup.bash
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

## 题二：待实现

输入为 `inputs/q2_demo_input.jsonl`，正式输出为 `answers/q2_demo_output.jsonl`。目前仅保留[开发说明](src/question2_project/README.md)，没有可执行入口。

## 题三：桥接诊断（不是触碰作答）

输入为 `inputs/q3_demo_input.jsonl`，正式输出为 `answers/q3_demo_output.jsonl`。目前只可运行[桥接探针](src/question3_project/README.md)；它不执行触碰、不改写官方场景、不生成作答。

## 提交材料

- [`assets/`](assets/README.md)：提交用的完整运行日志。当前桥接基线仅供诊断，不代替题三日志。
- [`video/`](video/README.md)：题一 RViz 和题三 Isaac Sim 视频均为必交；单个文件不超过 **30,000,000 字节**。视频完成后在此 README 填写实际文件名。
- [`docs/提交自查.md`](docs/提交自查.md)：正式提交前核对清单。请填写参赛信息和实际完成的亮点，将本目录改名为报名姓名后放入独立 AtomGit 作品仓库，不要上传外层开发工作区。
