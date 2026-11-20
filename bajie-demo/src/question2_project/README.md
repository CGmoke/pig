# 题二 · 正向运动学与坐标变换

从官方 URDF 提取 `base_footprint → objects_link` 的关节链，按关节 `origin/axis/limit` 计算刚体变换，输出夹爪在底盘系下的位姿，以及物体在夹爪系下的位姿。求解本身只依赖 Python 与 NumPy，不启动 ROS；另有一个基于 `robot_state_publisher` + TF2 的独立复核脚本。

## 目录结构

| 路径 | 作用 |
| --- | --- |
| [`fk.py`](fk.py) | `BajieChain`：解析 URDF 关节链，`T_A_B` 表示把 B 系坐标映射到 A 系；含 RPY/四元数互转、`rigid_inverse` |
| [`solve_q2.py`](solve_q2.py) | 命令行入口：逐行读取 JSONL，校验关节与位姿，原子写出作答并保持输入顺序 |
| [`verify_tf.py`](verify_tf.py) | 独立复核：运行官方 `robot_state_publisher`，发布关节状态与物体坐标系，在同一时间戳查 TF2 |
| [`tests/test_q2.py`](tests/test_q2.py) | 8 项测试：题面基准值、升降不变性、弯曲臂姿态重构、万向锁、批处理契约 |

## 环境与依赖

- Ubuntu 22.04 / ROS 2 Humble（仅复核需要 ROS；求解只需 Python 3.10+ 与 NumPy）。
- 官方 URDF 仓库 <https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf>，入口 `urdf/bajie_robot.urdf.xacro`。
- 验证环境：Python 3.10.12、NumPy 1.21.5；复核另需 `rclpy`、`sensor_msgs`、`geometry_msgs`、`tf2_ros`、`robot_state_publisher`。

## 准备展开后的 URDF

`BajieChain.from_urdf` 只接受**已展开**的 URDF（不能直接传 Xacro）。若还没有展开文件：

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export BAJIE_URDF_DIR=/你的模型目录/ecovacs-bajie-urdf
q2_model_dir="$(mktemp -d)"
(cd "$q2_model_dir" && colcon build --base-paths "$BAJIE_URDF_DIR" \
  --packages-select bajie_robot --cmake-args -DBUILD_TESTING=OFF)
source "$q2_model_dir/install/setup.bash"
export BAJIE_URDF="$q2_model_dir/bajie_robot.urdf"
xacro "$BAJIE_URDF_DIR/urdf/bajie_robot.urdf.xacro" -o "$BAJIE_URDF"
```

已有展开文件时，直接把 `BAJIE_URDF` 设为其绝对路径。计算不需要 ROS 图或模型网格；仅复核需要 ROS 2，建议使用没有其他机器人演示的 ROS 域。

## 运行

以下命令从**作品目录 `bajie-demo/`**执行：

```bash
set -o pipefail
python3 src/question2_project/solve_q2.py --urdf "$BAJIE_URDF" \
  --input inputs/q2_demo_input.jsonl --output answers/q2_demo_output.jsonl 2>&1 | tee assets/q2_run.log
```

脚本先计算完所有行再写文件：先写入输出目录下的临时目录，成功后 `replace`，因此解析或计算失败时不会覆盖已有作答，也不会留下半份答案；`--input` 与 `--output` 不能是同一文件。

### 输入格式

一行一个对象。`joint_state` 必须包含全部 10 个关节，`scene_object_in_base_footprint` 为物体在底盘系的位姿：

```json
{"id":"demo_zero",
 "joint_state":{"lift_joint":0.0,"pitch_joint":0.0,"joint1":0.0,"joint2":0.0,"joint3":0.0,"joint4":0.0,"joint5":0.0,"joint6":0.0,"arm_finger_joint1":0.0,"arm_finger_joint2":0.0},
 "scene_object_in_base_footprint":{"xyz":[0.5,-0.5,0.3],"rpy":[0.0,0.0,0.0]}}
```

`xyz` 单位为米，`rpy` 单位为弧度；数值必须是有限数，`NaN`/`Inf` 会被拒绝并标注行号。

### 输出格式

```json
{"id":"demo_zero",
 "objects_link_in_base_footprint":{"xyz":[...],"rpy":[...],"quat_xyzw":[...]},
 "scene_object_in_objects_link":{"xyz":[...],"rpy":[...],"quat_xyzw":[...]}}
```

四元数顺序为 `[x, y, z, w]`；`q` 与 `-q` 表示同一旋转，复核时用旋转角误差而不是逐项比较四元数。

## 原理：先找链，再换坐标系

1. 从 `objects_link` 沿父关节回溯到 `base_footprint`，再倒序相乘。每段先带入模型给出的 `origin`，再在关节局部系中沿 `axis` 平移（prismatic）或旋转（revolute/continuous），即 `T_origin @ T_motion(q)`。头部和手指是旁支，值要读取但不参与这条链；固定关节没有输入量，仍然贡献位置和朝向。
2. 用 `T_A_B` 表示把 **B 系坐标换到 A 系**。要求物体用夹爪系表示，就计算 `T_gripper_object = inverse(T_base_gripper) @ T_base_object`。位置部分是 `R_base_gripperᵀ @ (p_base_object − p_base_gripper)`：先减去夹爪原点，再把方向换到夹爪轴上，不能只做减法。
3. 旋转矩阵按 `Rz(yaw) @ Ry(pitch) @ Rx(roll)` 构造。矩阵转 RPY 时区分常规情况与俯仰接近 `±π/2` 的万向锁情况；转四元数时按最大对角元选主分支，并统一 `w ≥ 0`。

## 独立复核

```bash
Q2_URDF="$BAJIE_URDF" PYTHONPATH="src/question2_project${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover -s src/question2_project/tests -v

# 在已加载 Humble 的终端运行；不另开 RViz，不启动 Isaac Sim。
ROS_LOCALHOST_ONLY=1 python3 src/question2_project/verify_tf.py --urdf "$BAJIE_URDF" \
  --input inputs/q2_demo_input.jsonl
```

`verify_tf.py` 用独立的 `robot_state_publisher` 进程（`frame_prefix` 为 `q2_verify/`，关节话题 `/q2_verify/joint_states`）生成参考结果，再用求解器结果与其比较，阈值 `1e-8 m` / `1e-7 rad`。复核用例共三组：正式零位、仅 `lift_joint +0.1 m`、弯曲臂且物体带 `rpy=[0.2,-0.3,0.5]`；后两组在内存中构造，不写入正式作答。

正式输入仅一行 `demo_zero`。Humble 的 KDL 会对根连杆惯量给出警告；这里不计算动力学，不为消除警告而修改官方模型。完整日志见 [`assets/q2_humble_verify.log`](../../assets/q2_humble_verify.log) 与 [`assets/q2_run.log`](../../assets/q2_run.log)。

## 动手练习：只升高夹爪 0.1 m

复制正式输入到临时文件，只把 `lift_joint` 从 `0.0` 改为 `0.1`，然后将求解命令的 `--input` 和 `--output` 都指向临时文件，不覆盖正式提交文件。

| 要观察的量 | 零位 | 升高 0.1 m 后 |
| --- | --- | --- |
| 夹爪相对底盘 `xyz`（m） | `[1.103715, -0.062685, 0.629136]` | `[1.103715, -0.062685, 0.729136]` |
| 夹爪相对底盘 `rpy`（rad，近似） | `[0, 0.000832, 0]` | 不变 |
| 物体相对夹爪 `xyz`（m） | `[-0.603441, -0.437315, -0.329638]` | `[-0.603358, -0.437315, -0.429638]` |

物体在底盘系中没有移动。为什么它在夹爪系中不仅降低，X 还略微增加？夹爪有约 `0.000832 rad` 的俯仰，底盘系里的向下位移换到夹爪系以后会产生 `0.1 × sin(0.000832) ≈ 0.0000832 m` 的正 X 分量。这能检验你是否真的做了坐标轴变换。

## 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| `请提供含 ... 的已展开 URDF，而非 Xacro` | 传入了 `.xacro` 或缺少 `base_footprint`/`objects_link`；先用 `xacro` 展开 |
| `joint_state 缺少 xxx` | 输入未包含全部 10 个关节，或关节名拼写不符 |
| `URDF 连杆 xxx 有多个父关节` / 找不到链 | 模型被误改，存在重复父关节或链断开 |
| `TF 复核失败：找不到正式零位示例 demo_zero` | `--input` 指错，或输入中 `id` 不是 `demo_zero` |
| `未收到配套 TF` | ROS 域被其他演示占用，或 `robot_state_publisher` 提前退出；用 `ROS_LOCALHOST_ONLY=1` 与独立域重试 |
| 警告根连杆惯量 | Humble KDL 的已知提示，不影响本静态计算，不要改官方模型 |

## 注意事项

- 输出保持输入的顺序与 `id`；追加多行测例时无需改代码，但出错会整体中止且不覆盖旧作答。
- 题三复用本目录的 `fk.py`，修改它会影响题三的规划与作答，改动前先跑两题的测试。
- 本目录不含 ROS 包，不能 `colcon build`；它只是可被测试与题三直接导入的 Python 源码目录。
