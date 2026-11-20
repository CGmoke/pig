# 八界杯工作区（bajie_ws）

ROS 2 Humble + Isaac Sim 4.5 的参赛工作区。官方要求的组合（Ubuntu 22.04 / Humble / Isaac Sim 4.5）
全部封装在容器 `bajie/sim:humble45` 里，宿主 Ubuntu 24.04 不参与编译，只负责显卡驱动与 Docker。

## 目录

```
bajie_ws/
├── src/
│   ├── bajie_robot/     # 官方 URDF 包（urdf/ meshes/ calib/ launch/ rviz/）
│   ├── bajie_sim/       # 官方仿真资源（scene/ machine/ models/）+ 本工作区补的 package.xml
│   └── bajie_tasks/     # 自己的任务包：t1_move / joint_probe
├── docker/              # Dockerfile.sim（Isaac Sim 4.5 + Humble）、compose.yaml
├── scripts/             # 环境体检、启动、URDF 展开、场景体检、桥接探针
├── docs/                # 环境搭建与踩坑记录
├── logs/                # 每次运行的日志与报告（诊断证据都在这）
├── generated/           # xacro 展开产物 bajie_robot.urdf
└── .ov-cache/ .ov-data/ # Isaac Sim 着色器/扩展缓存（挂进容器，冷启动从 4 分钟降到几十秒）
```

## 三条命令（都在本目录下执行）

```bash
# 1) 编译 ROS 包（在容器里跑，宿主不需要装 ROS）
docker run --rm -v $PWD:/workspace -w /workspace bajie/sim:humble45 \
  bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'

# 2) 展开官方 xacro → generated/bajie_robot.urdf
docker run --rm -v $PWD:/workspace bajie/sim:humble45 \
  bash -c 'source /opt/ros/humble/setup.bash && bash /workspace/scripts/expand_urdf.sh'

# 3) 无头跑官方场景并实测 ROS 话题
docker run --rm --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v $PWD:/workspace -v $PWD/.ov-cache:/root/.cache/ov -v $PWD/.ov-data:/root/.local/share/ov \
  bajie/sim:humble45 bash -c 'source /opt/ros/humble/setup.bash && \
    /isaac-sim/python.sh /workspace/scripts/run_scene_ros.py --duration 8'
```

## 环境状态（2026-11-20 实测）

| 项           | 状态                                                                                    |
| ------------ | --------------------------------------------------------------------------------------- |
| 驱动         | 580.178.04（**必须 ≤580**，595.x 会让 RTX 渲染器在 `librtx.scenedb` 段错误）   |
| 镜像         | `bajie/sim:humble45`，含 Humble + rmw_fastrtps_cpp + ROS 2 bridge 预编译库            |
| GPU 穿透     | ✅ 容器内 `nvidia-smi` 正常                                                           |
| 官方场景加载 | ✅ `abr_scene.usd` 打开，1 个 Articulation Root / 34 关节 / 4 相机 + 1 Lidar          |
| 时间轴       | ✅ 脚本 play 生效（仿真时间实测推进 0 → 87s）                                          |
| ROS 桥接     | ⚠️ **场景内没有 ROS2 图节点**，需自己挂 `machine/component/omni_graphs/*.usd` |

## 当前卡点与下一步

`abr_scene.usd` 里**没有任何 ActionGraph / ROS2 节点**（`scripts/diag_graphs.py` 已验证）。
官方把桥接图拆成了独立 USD，需要逐张挂到场景里：

```
machine/component/omni_graphs/
├── clock.usd                # /clock
├── transform_tree.usd       # /tf /tf_static
├── arm_ctrl.usd             # 关节指令订阅
├── different_ctrl.usd       # 底盘控制
├── lidars.usd               # 激光雷达
├── ros_camera_arm.usd       # 臂部相机
├── ros_cameras_tof.usd      # ToF 相机
└── ros_cameras_upper.usd    # 头部相机
```

下一步：写 `scripts/mount_graphs.py`，在场景里 reference 这些图并另存为 `scene/abr_scene_ros.usd`，
然后 `joint_probe` 就能收到 `/joint_states`，T1 才具备数据基础。

## 提交合规

- 模型必须基于官方 URDF：`generated/bajie_robot.urdf` 由官方 xacro 直接展开，未改几何；
- 导入 Isaac Sim 时**禁止勾选 Merge Fixed Joints**（31 个关节里有 21 个 fixed，勾了就等于改了模型）；
- 用 `scripts/check_urdf_origin.py` 逐条比对 origin，输出可直接作为合规证据；
- `logs/smoke_report.txt`（场景结构）与 `logs/diag_graphs.txt`（桥接状态）同样可作为证据材料。
