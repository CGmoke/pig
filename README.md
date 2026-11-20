# 八界杯工作区（bajie_ws）

ROS 2 Humble + Isaac Sim 4.5 的参赛工作区。官方要求的组合（Ubuntu 22.04 / Humble / Isaac Sim 4.5）
全部封装在容器 `bajie/sim:humble45` 里，宿主 Ubuntu 24.04 不参与编译，只负责显卡驱动与 Docker。

## 参赛提交目录

[`bajie-demo/`](bajie-demo/README.md) 是独立于本开发环境的提交目录，按赛题分题组织源码、输入、答案、日志和视频。构建缓存、现场排障材料、官方大体积模型留在开发侧。题一是现有实现的提交副本，代码更新后需同步并重新生成对应答案和日志；题二和题三的后续提交材料放在各自的子目录。正式提交时，只将该目录放到独立 AtomGit 作品仓库根目录并改为报名姓名。

## 目录

```
bajie_ws/
├── src/
│   ├── bajie_robot/     # 官方 URDF 包（urdf/ meshes/ calib/ launch/ rviz/）
│   ├── bajie_sim/       # 官方仿真资源（scene/ machine/ models/）+ 本工作区补的 package.xml
│   └── bajie_tasks/     # 自己的任务包：t1_move / joint_probe
├── docker/              # Dockerfile.sim（Isaac Sim 4.5 + Humble）、Dockerfile.rviz、compose.yaml
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

## RViz 可视化（Docker）

`Dockerfile.sim` 里 rviz2 默认没装（约 1.2G），可视化用单独的 overlay 镜像：

```bash
# 1) 构建（只需一次，基于本地 bajie/sim:humble45，几秒）
docker build -f docker/Dockerfile.rviz -t bajie/sim:humble45-rviz docker

# 2) 启动
xhost +si:localuser:root
docker run --rm -it --name bajie-rviz \
  --network host --ipc host --shm-size=2gb \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:ro" \
  -e XAUTHORITY=/root/.Xauthority \
  -v $PWD:/workspace -w /workspace \
  bajie/sim:humble45-rviz \
  bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
            ros2 launch bajie_robot display_bajie_robot_urdf.launch.py use_sim_time:=false'
```

- `use_sim_time:=false`：单独看模型时没有 `/clock`，必须关掉，否则 TF 时间戳不对。
- 镜像 `/etc/profile.d` 里自动 source 的是 `/workspace/bajie_ws/install`，而仓库挂在 `/workspace`，所以要手动 `source /workspace/install/setup.bash`。
- 要和 Isaac Sim 同图：两边都用 `--network host` 且 `ROS_DOMAIN_ID` 一致（默认 42）。

已知坑：

| 现象                             | 原因 / 处理                                                                                                                            |
| -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| UI 正常、3D 区域纯黑             | NVIDIA 的 GL 线程优化与 Ogre GLX 冲突。`Dockerfile.rviz` 已设 `ENV __GL_THREADED_OPTIMIZATIONS=0`，旧镜像需重建、旧容器需重启      |
| 构建报 `held broken packages`  | 基础镜像的 `libbrotli1` 是 sury 的 1.1.0，与 jammy 的 `libbrotli-dev` 冲突；Dockerfile.rviz 已用 `--allow-downgrades` 降回 1.0.9 |
| `executable 'rviz2' not found` | 用了 `bajie/sim:humble45`，要换 `bajie/sim:humble45-rviz`                                                                          |
| 模型纯白/灰                      | 官方 mesh 无贴图，正常                                                                                                                 |

## 环境状态（2026-11-20 实测）

| 项           | 状态                                                                                                     |
| ------------ | -------------------------------------------------------------------------------------------------------- |
| 驱动         | 580.178.04（**必须 ≤580**，595.x 会让 RTX 渲染器在 `librtx.scenedb` 段错误）                    |
| 镜像         | `bajie/sim:humble45`，含 Humble + rmw_fastrtps_cpp + ROS 2 bridge 预编译库                             |
| GPU 穿透     | ✅ 容器内 `nvidia-smi` 正常                                                                            |
| 官方场景加载 | ✅ `abr_scene.usd` 打开，1 个 Articulation Root / 34 关节 / 4 相机 + 1 Lidar                           |
| 时间轴       | ✅ 脚本 play 生效（仿真时间实测推进 0 → 87s）                                                           |
| ROS 桥接     | ⚠️ **场景内没有 ROS2 图节点**，需自己挂 `machine/component/omni_graphs/*.usd`                  |
| RViz2        | ✅ `bajie/sim:humble45-rviz`，NVIDIA 渲染黑屏已用 `__GL_THREADED_OPTIMIZATIONS=0` 修复（镜像内固化） |

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

### 已实测的结论（2026-11-20）

| 尝试                                             | 结果                                                                     |
| ------------------------------------------------ | ------------------------------------------------------------------------ |
| 直接 reference 这 8 张图 + 重连 12 条 targetPrim | 挂上了、关系全对，但**一帧数据都不发**                             |
| 把图根 `OmniGraph` 改判成 `ActionGraph`      | 仍不发                                                                   |
| **用 `og.Controller` 现场搭最小图**      | ✅ 成功：25 秒收到 `/clock` 183 条、`/joint_states` 182 条，1:1 实时 |

结论：**别挂官方图，自己搭最小桥接图**（`scripts/build_bridge.py`）。
搭之前必须 `set_extension_enabled_immediate("isaacsim.ros2.bridge", True)`，
否则报 `unrecognized type 'isaacsim.ros2.bridge.ROS2Context'`。

### 12 个可控关节（实测 /joint_states）

```
底盘   wheel_left_joint  wheel_right_joint
升降   lift_joint
头部   pitch_joint
机械臂 joint1 … joint6
夹爪   arm_finger_joint1  arm_finger_joint2
```

> 注意：URDF 里轮子是 `fixed`（静态 TF 用），但场景 USD 里是 `PhysicsRevoluteJoint`——
> 所以底盘在仿真里真能跑，这一点以 USD 为准。

### 已知未闭环

`scripts/serve_sim.py`（仿真常驻 + 外部节点下发）能注入图、能播放，但**话题列表仍为空**；
而 `build_bridge.py`（开场景+建图+播放+测试同一进程）确定可用。
T1 的稳定跑法应走**单进程一体化**：一个脚本里完成"开场景 → 建图 → 播放 → 驱动 → 记录"。

## 提交合规

- 模型必须基于官方 URDF：`generated/bajie_robot.urdf` 由官方 xacro 直接展开，未改几何；
- 导入 Isaac Sim 时**禁止勾选 Merge Fixed Joints**（31 个关节里有 21 个 fixed，勾了就等于改了模型）；
- 用 `scripts/check_urdf_origin.py` 逐条比对 origin，输出可直接作为合规证据；
- `logs/smoke_report.txt`（场景结构）与 `logs/diag_graphs.txt`（桥接状态）同样可作为证据材料。
