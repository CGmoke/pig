# 题二 · 正向运动学与坐标变换

- [`fk.py`](fk.py)：从官方 URDF 读取关节原点、类型与轴，计算刚体变换。
- [`solve_q2.py`](solve_q2.py)：逐行读取 JSONL，生成[正式作答](../../answers/q2_demo_output.jsonl)。保持 `id` 和输入顺序，出错时注明行号且不覆盖已有作答。
- [`verify_tf.py`](verify_tf.py)：运行 `robot_state_publisher`，向 ROS 2 发布关节状态和物体坐标系，在同一时间戳查 TF2，不用求解器的矩阵运算生成参考结果。

## 运行与复核

以下命令从**作品目录 `bajie-demo/`**执行。环境为 Ubuntu 22.04 / ROS 2 Humble，依赖 `colcon`、`ament_cmake`、`xacro`、`rclpy`、`sensor_msgs`、`geometry_msgs`、`tf2_ros`、`robot_state_publisher` 和 `python3-numpy`。验证环境：Python 3.10.12、NumPy 1.21.5。官方模型获取方式见[作品 README](../../README.md)。

如果还没有已展开的官方 URDF，设置 `BAJIE_URDF_DIR` 为官方 ROS 包的绝对路径（目录中应有 `package.xml` 和 `urdf/`），执行：

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

已有官方展开文件时，直接将 `BAJIE_URDF` 设为其绝对路径。计算不需要 ROS 图或模型网格；仅复核需要 ROS 2。建议选用没有其他机器人演示的 ROS 域。

```bash
set -o pipefail
python3 src/question2_project/solve_q2.py --urdf "$BAJIE_URDF" \
  --input inputs/q2_demo_input.jsonl --output answers/q2_demo_output.jsonl 2>&1 | tee assets/q2_run.log

Q2_URDF="$BAJIE_URDF" PYTHONPATH="src/question2_project${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover -s src/question2_project/tests -v

# 在已加载 Humble 的终端运行；不另开 RViz，不启动 Isaac Sim。
ROS_LOCALHOST_ONLY=1 python3 src/question2_project/verify_tf.py --urdf "$BAJIE_URDF" \
  --input inputs/q2_demo_input.jsonl
```

正式输入仅一行 `demo_zero`。两组复核用的扩展姿态由验证脚本在内存中构造，不写入正式作答。Humble 的 KDL 对根连杆惯量给出警告；这里不计算动力学，不为消除警告而修改官方模型。详见[完整运行与复核日志](../../assets/q2_humble_verify.log)。

## 原理：先找链，再换坐标系

1. 从 `objects_link` 沿父关节往回找到 `base_footprint`，反过来逐段相乘。每段先带入模型给出的 `origin`，再在关节局部系中沿 `axis` 移动或旋转，即 `T_origin @ T_motion(q)`。头部和手指是旁支，所以它们的值要读取，但不参与这条链。固定关节没有输入量，仍然贡献位置和朝向。
2. 用 `T_A_B` 表示把 **B 系的坐标换到 A 系**。要求物体用夹爪系表示，就计算 `T_gripper_object = inverse(T_base_gripper) @ T_base_object`。位置部分是 `R_base_gripperᵀ @ (p_base_object − p_base_gripper)`：先减去夹爪原点，再把方向换到夹爪的轴上，不能只做减法。
3. `xyz` 用米，`rpy` 用弧度；旋转矩阵按 `Rz(yaw) @ Ry(pitch) @ Rx(roll)` 构造。输出四元数为 `[x, y, z, w]`；`q` 和 `-q` 表示同一旋转，所以复核使用旋转角误差，而不是逐项比较四元数。

## 动手练习：只升高夹爪 0.1 m

复制正式输入到临时文件，只把 `lift_joint` 从 `0.0` 改为 `0.1`，然后将求解命令的 `--input` 和 `--output` 都指向临时文件，不覆盖正式提交文件。

| 要观察的量                        | 零位                                  | 升高 0.1 m 后                         |
| --------------------------------- | ------------------------------------- | ------------------------------------- |
| 夹爪相对底盘 `xyz`（m）         | `[1.103715, -0.062685, 0.629136]`   | `[1.103715, -0.062685, 0.729136]`   |
| 夹爪相对底盘 `rpy`（rad，近似） | `[0, 0.000832, 0]`                  | 不变                                  |
| 物体相对夹爪 `xyz`（m）         | `[-0.603441, -0.437315, -0.329638]` | `[-0.603358, -0.437315, -0.429638]` |

物体在底盘系中没有移动。为什么它在夹爪系中不仅降低，X 还略微增加？夹爪有大约 `0.000832 rad` 的俯仰，底盘系里的向下位移换到夹爪系以后会产生一个约 `0.1 × sin(0.000832) = 0.0000832 m` 的正 X 分量。这能检验你是否真的做了坐标轴变换。
