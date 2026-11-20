# 题一 · 关节分步驱动与 RViz 展示

用 `q1_drive` 节点按 `lift → head → arm → gripper_open → gripper_close` 的顺序驱动官方模型的 10 个关节，每阶段用 smoothstep 在 `move_sec` 内插值并以 30 Hz 发布 `/joint_states`；`robot_state_publisher` 将其转换为 TF，RViz2 的 RobotModel 负责渲染。所有阶段在播放前都要通过基于官方 URDF 几何的净空预检。

本目录是一个完整的 `ament_python` 包 `bajie_tasks`，依赖官方 `bajie_robot` 模型包。

## 目录结构

| 路径                                                      | 作用                                                                             |
| --------------------------------------------------------- | -------------------------------------------------------------------------------- |
| [`bajie_tasks/q1_drive.py`](bajie_tasks/q1_drive.py)     | 主节点：读取 JSONL 测例、采样目标位姿、分阶段插值发布、写作答                    |
| [`bajie_tasks/q1_safety.py`](bajie_tasks/q1_safety.py)   | `ClearanceModel`：解析 URDF 的 visual/collision 几何，做整段轨迹的保守净空证明 |
| [`launch/q1_rviz.launch.py`](launch/q1_rviz.launch.py)   | 一体化启动 `robot_state_publisher` + `q1_drive` + `rviz2`                  |
| [`test/test_q1_clearance.py`](test/test_q1_clearance.py) | 预检回归测试：旧 seed 7 触地、机身间距、区间中间危险、0–127 种子全阶段          |
| [`package.xml`](package.xml) / [`setup.py`](setup.py)   | 包元数据与入口点 `q1_drive = bajie_tasks.q1_drive:main`                        |

## 环境与依赖

- Ubuntu 22.04 + ROS 2 Humble（`rclpy`、`sensor_msgs`、`launch`、`launch_ros`、`ament_index_python`、`xacro`、`robot_state_publisher`、`rviz2`、`python3-numpy`）。
- 官方 URDF 包 `bajie_robot`，入口为 `urdf/bajie_robot.urdf.xacro`，并提供 `rviz/display.rviz`。
- 需要图形桌面才能看到 RViz；无桌面时可只运行节点生成作答。

## 构建

以下命令从**作品目录 `bajie-demo/`**执行：

```bash
export BAJIE_URDF_DIR=/你的模型目录/ecovacs-bajie-urdf
source /opt/ros/humble/setup.bash
colcon build --base-paths src/question1_project "$BAJIE_URDF_DIR" \
  --packages-select bajie_robot bajie_tasks --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 运行

```bash
ros2 launch bajie_tasks q1_rviz.launch.py \
  input_file:="$PWD/inputs/q1_demo_input.jsonl" \
  output_file:="$PWD/answers/q1_demo_output.jsonl"
```

预期表现：升降 → 保持 5 s → 头部 → 保持 5 s → 机械臂 → 保持 5 s → 夹爪张开 → 夹爪闭合；终端每个阶段输出一行净空下界，最后写出一行作答。

### 节点、参数与话题

| 名称                    | 类型                               | 说明                                                               |
| ----------------------- | ---------------------------------- | ------------------------------------------------------------------ |
| `q1_drive`（节点）    | —                                 | 唯一发布 `/joint_states` 的节点                                  |
| `input_file`          | 参数（string）                     | 默认 `inputs/q1_demo_input.jsonl`；相对路径基于启动目录          |
| `output_file`         | 参数（string）                     | 默认 `answers/q1_demo_output.jsonl`                              |
| `publish_hz`          | 参数（double）                     | 默认 `30.0`，即插值与保持时的发布频率                            |
| `robot_description`   | 参数（string）                     | 留空时节点自行用 xacro 展开官方模型；launch 会显式传入             |
| `/joint_states`       | 话题（`sensor_msgs/JointState`） | 本节点发布，默认 QoS：RELIABLE / VOLATILE / KEEP_LAST 10           |
| `/tf`、`/tf_static` | 话题                               | 由 `robot_state_publisher` 依据 `/joint_states` 与固定关节发布 |

`q1_drive` 不调用 `create_rate()`，而是用 `time.sleep(1/publish_hz)` 控制节拍，避免未 `spin` 时定时器阻塞。launch 中**不启动** `joint_state_publisher_gui`，否则两个节点会同时发布 `/joint_states`，RViz 中模型会抖动。

### 输入与输出

输入一行一个 JSON 对象，支持字段：`id`、`step`（`all` 或单个阶段名）、`seed`、`move_sec`（默认 3.0）、`pause_sec`（默认 5.0）。

```json
{"id": "q1_all_seed7", "step": "all", "seed": 7, "move_sec": 3.0, "pause_sec": 5.0}
```

输出保持输入顺序与 `id`，包含每个阶段结束时的 10 关节快照：

```json
{"id":"q1_all_seed7","step":"all",
 "keyframes":[{"name":"lift","joint_state":{"lift_joint":0.183626,"pitch_joint":0.0, "...":"..."}}],
 "final_joint_state":{"lift_joint":0.183626,"...":"..."}}
```

`seed` 决定目标位姿：升降、头部、`joint1`–`joint6` 与夹爪开度各自在预设范围内用 `random.Random(seed)` 取值，因此同一 seed 可复现。机械臂目标整体朝前上方抬展，避免原始演示中较大的负向 `joint2` 把腕部压向地面。多行测例之间若上一例结束时不在零位，会先插入一段 `reset`，该段同样要经过预检。

## 净空预检原理

1. 用 xacro 展开官方模型，解析每个 link 的 `visual` 与 `collision`；网格只支持官方使用的二进制 STL，盒体/圆柱/球体按解析几何处理，再按 `origin` 变换成该连杆坐标系下的局部包围盒。
2. 从 `base_footprint` 建树，正向计算各连杆位姿；包围盒在世界中的半尺寸取 `|R| @ half`，得到保守 AABB。
3. 检查两类间距：机械臂各段到 `base_footprint` 下 `z=0` 地面至少 **5 cm**，机械臂各段到非相连机身至少 **2 cm**；`arm1_link` 与固定安装座 `arm_base_link` 的固有连接面不计入。
4. 不只看端点：对关节插值段二分检查（最多 12 层），用“最远角半径 × |Δq|（旋转）/ |Δq|（升降）”估计该区间内任一点的最大位移，只有当位移上界仍满足净空时才认为整段安全。
5. 预检失败抛 `UnsafeMotion`，节点记录原因（如“对地面净空不足 0.031 m”）、以非零码退出，并且**不写该例作答**；已完成的测例会先落盘。

这是 RViz 模型上的几何验证，不是物理接触仿真。

## 验证与测试

先构建并加载环境（需要已安装 `bajie_robot`）：

```bash
python3 -m unittest discover -s src/question1_project/test -v
```

覆盖：seed 7 旧姿态触地、机身间距独立于地面、两个安全端点之间的中段不安全、seed 0–127 全阶段及回零均满足阈值。

运行时可辅助检查：

```bash
ros2 topic hz /joint_states                 # 约 30 Hz
ros2 topic echo /joint_states --once        # 10 个关节名与位置
ros2 run tf2_ros tf2_echo base_footprint objects_link
ros2 run tf2_tools view_frames             # 检查关节树无断链
```

参考实测日志见 [`assets/q1_humble_headless.log`](../../assets/q1_humble_headless.log)：五个阶段的地面净空下界为 0.514 / 0.698 / 0.670 / 0.679 / 0.679 m，机身净空下界为 0.021 / 0.044 / 0.022 / 0.035 / 0.035 m。

## 常见问题

| 现象                                     | 原因与处理                                                                                                     |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `Package 'bajie_robot' not found`      | 未构建或未 `source install/setup.bash`；确认 `BAJIE_URDF_DIR` 指向含 `package.xml` 与 `urdf/` 的官方包 |
| RViz 中模型不动或抖动                    | 同时存在 `joint_state_publisher_gui`；关闭它，只保留 `q1_drive`                                            |
| 启动即报 xacro/解析错误                  | 官方模型路径不正确，或 `bajie_robot` 安装目录缺少 `urdf/`、`rviz/display.rviz`                           |
| `UnsafeMotion: ... 净空不足`           | 该 seed 的目标姿态不可用；换 seed，或检查是否误改了 URDF 的 `origin`/几何                                    |
| 写作答时报 `EACCES: permission denied` | `answers/` 目录属主为 root；把它改回当前用户，见根目录说明                                                   |
| 找不到 `inputs/...jsonl`               | 参数用了相对路径，而启动目录不是 `bajie-demo/`；改用绝对路径                                                 |

## 注意事项

- 题一必交 RViz 演示视频；视频录制与作答生成是两件事，作答只需程序正常跑完。
- 不要用 `sudo colcon build`，否则 `build/ install/ log/` 会变成 root 所有，后续构建和删除都会失败。
- 修改官方 URDF 几何或关节 `origin` 会同时影响题一净空判断与题二正运动学结果。
