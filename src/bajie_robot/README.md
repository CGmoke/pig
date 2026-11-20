# ecovacs-bajie-urdf

八界机器人 ROS 2 描述包（`bajie_robot`）。提供整机 URDF/Xacro、网格、传感器外参标定和 RViz 可视化，用于 TF 发布、运动规划、碰撞检测与仿真接入。

## 关于八界

八界是科沃斯面向具身智能开发者推出的全栈开源机器人平台，集成机械臂、夹爪、升降机构、多模态感知系统与计算平台。软件侧提供 ROS 2 接口以及 C++ / Python SDK，覆盖建图导航、视觉感知、目标搜索、机械臂控制、抓取放置、语音交互和状态管理。

本仓库聚焦机器人描述与可视化，不包含 SDK 与任务编排代码。应用开发请参考八界 SDK 交付包（`ecovacs_bajie_sdk`）。

## 机器人结构

整机从 `base_footprint` 起链，由移动底盘、升降柱、俯仰头部、机械臂与夹爪组成。SolidWorks 装配体导出后经 Xacro 组装，传感器外参由 `calib/calibration.yaml` 注入。

```text
base_footprint
├── line_laser_down_link          # 底部线激光（标定）
└── base_link                     # 移动底盘
    ├── wheel_left / wheel_right  # 差速驱动轮
    ├── caster_*                  # 前左 / 前右 / 后随动轮
    ├── base_scan                 # 底盘激光雷达
    ├── infrared_link             # 后向红外
    └── lift_base_link
        └── lift_link             # 升降机构
            ├── head_pitch_link   # 头部俯仰
            │   ├── berxel_upper_base_link   # 头部 RGBD（SDK HEAD）
            │   ├── head_wide_angle_link     # 头部广角
            │   └── tof_top_link → royale_camera_optical_frame
            └── arm_base_link
                └── joint1 … joint6          # 机械臂
                    ├── gripper_link
                    │   ├── gripper_left / right_link
                    │   └── objects_link     # 抓取点
                    └── berxel_arm_base_link # 腕部 RGBD（SDK ARM）
```

当前可视化模型中，轮系与随动轮关节类型为 `fixed`，用于静态 TF 与外形展示，不驱动滚动仿真。

### 运动机构

- **升降机构**：调节上半身高度，适配不同作业位置。
- **头部俯仰**：调整感知方向，配合观察与目标定位。
- **机械臂**：提供末端运动能力，支持操作任务的规划与模型展示。
- **夹爪**：通过主关节与随动关节实现开合，`objects_link` 用于表示抓取参考点。

关节限位、几何尺寸与坐标偏移统一在 URDF/Xacro 中维护，本文不展开具体参数。实机控制请结合设备配置与 SDK 文档。

### 感知与相机

SDK `eco_captureImages` 使用两类 RGBD：`CameraType.HEAD` 与 `CameraType.ARM`。头部图像位姿在 `map` 下，手臂图像位姿在 `base_footprint` 下。本模型中对应连杆如下。

| 连杆 / 坐标系 | 安装位置 | 用途 |
|:---|:---|:---|
| `berxel_upper_base_link` | 头部俯仰连杆 | 头部 RGBD，对应 SDK `HEAD` |
| `berxel_arm_base_link` | 腕部 `arm6_link` | 手臂 RGBD，对应 SDK `ARM` |
| `head_wide_angle_link` | 头部 | 广角相机 |
| `tof_top_link` / `royale_camera_optical_frame` | 头部 | 顶部 ToF |
| `base_scan` | 底盘前方 | 2D 激光雷达 |
| `line_laser_down_link` | `base_footprint` | 底部线激光 |
| `infrared_link` | 底盘后方 | 红外 |

`urdf/berxel_upper_base_link.urdf.xacro` 与 `urdf/berxel_arm_base_link.urdf.xacro` 定义了 RGB/Depth 光学坐标系，供相机驱动或规划栈按需 include。主模型 `bajie_robot.urdf.xacro` 目前只挂载 Berxel 基座连杆，光学坐标系需单独引入。

## 仓库结构

```text
ecovacs-bajie-urdf/
├── CMakeLists.txt
├── package.xml
├── calib/calibration.yaml          # 头部/腕部 RGBD、ToF、线激光外参
├── launch/display_bajie_robot_urdf.launch.py
├── meshes/                         # 视觉与碰撞 STL
├── rviz/display.rviz
└── urdf/
    ├── bajie_robot.urdf.xacro      # 整机入口
    ├── xacro_property.urdf.xacro   # 从 YAML 读取外参
    ├── gripper.urdf.xacro          # 夹爪
    ├── berxel_upper_base_link.urdf.xacro
    └── berxel_arm_base_link.urdf.xacro
```

## 标定

`xacro_property.urdf.xacro` 默认加载 `calib/calibration.yaml`，覆盖以下固定关节的 `xyz` / `rpy`：

- `head_rgbd_joint` → `berxel_upper_base_link`
- `gripper_rgbd_joint` → `berxel_arm_base_link`
- `tof_top_joint` → `tof_top_link`
- `line_laser_down_joint` → `line_laser_down_link`

仓库内标定配置为占位配置，接入实机前需替换为对应设备的标定结果。外参通过 YAML 统一维护，无需修改 URDF 本体；也可通过 `calibration_file` 指定其他标定文件。

## 依赖与构建

需要 ROS 2 工作空间，以及：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro`
- `launch_param_builder`（launch 中用于加载 Xacro）

将本包放入工作空间 `src/` 后编译：

```bash
cd <your_ws>
colcon build --packages-select bajie_robot
source install/setup.bash
```

## 可视化

```bash
ros2 launch bajie_robot display_bajie_robot_urdf.launch.py
```

会启动：

1. `robot_state_publisher`：展开 `urdf/bajie_robot.urdf.xacro` 并发布 TF
2. `joint_state_publisher_gui`：拖动升降、头部、臂与夹爪
3. `rviz2`：加载 `rviz/display.rviz`

用于核对连杆树、碰撞体、相机安装位置，以及标定后的传感器外参。

## 与 SDK 的对应关系

| 本模型 | SDK / 应用层 |
|:---|:---|
| `base_footprint` | 导航与手臂相机 `tf_goal` 参考系 |
| `lift_joint` | `eco_setRobotHeight` |
| `pitch_joint` | `eco_setRobotHead` |
| `joint1`–`joint6` + 夹爪 | 机械臂模式、位姿、抓放 |
| `berxel_upper_base_link` | `eco_captureImages(CameraType.HEAD)` |
| `berxel_arm_base_link` | `eco_captureImages(CameraType.ARM)` |
| `objects_link` | 抓取点 / TCP |

模型为感知定位、运动规划与抓取放置提供统一的几何描述，具体任务流程由 SDK 与应用层实现。

## 许可

包内 `package.xml` 许可声明待补充。请以项目实际开源协议为准。
