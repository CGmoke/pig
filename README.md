# 八界杯工作区（bajie_ws）

ROS 2 Humble + Isaac Sim 4.5 的参赛工作区。官方要求的组合（Ubuntu 22.04 / Humble / Isaac Sim 4.5）
全部封装在容器 `bajie/sim:humble45` 里，宿主机只负责 NVIDIA 驱动、Docker 和显示，不参与编译。

`bajie-demo/` 是独立的参赛提交目录，说明见 [bajie-demo/README.md](bajie-demo/README.md)。

---

## 1. 项目简介

本工作区提供三类能力：

| 能力               | 入口                   | 说明                                                                                         |
| ------------------ | ---------------------- | -------------------------------------------------------------------------------------------- |
| ROS 2 工作区编译   | `src/` + `colcon`  | `bajie_robot`（官方 URDF）、`bajie_sim`（官方场景资源）、`bajie_tasks`（题一任务节点） |
| Isaac Sim 场景运行 | `scripts/run_sim.sh` | `gui` / `livestream` / `headless` 三种模式                                             |
| 模型与桥接验证     | `scripts/*.py`       | 场景体检、URDF 展开、origin 合规比对、ROS 2 桥接探针                                         |

目录结构：

```text
bajie_ws/
├── src/
│   ├── bajie_robot/     # 官方 URDF 包（urdf/ meshes/ calib/ launch/ rviz/）
│   ├── bajie_sim/       # 官方仿真资源（scene/ machine/ models/）+ package.xml
│   └── bajie_tasks/     # 自己的任务包：q1_drive / joint_probe / t1_move
├── docker/              # Dockerfile.sim、Dockerfile.rviz、compose.yaml
├── scripts/             # 环境体检、启动、URDF 展开、场景体检、桥接探针
├── docs/                # 环境搭建与踩坑记录（细节以 docs/开发环境搭建.md 为准）
├── inputs/ answers/     # 题一输入与作答
├── logs/                # 每次运行的日志与报告（诊断证据都在这）
├── generated/           # xacro 展开产物 bajie_robot.urdf
└── .ov-cache/ .ov-data/ # Isaac Sim 着色器/扩展缓存（挂进容器，冷启动从 4 分钟降到几十秒）
```

---

## 2. 前置依赖

### 2.1 版本清单

| 类别   | 组件                         | 最低版本 / 要求           | 说明                                                          |
| ------ | ---------------------------- | ------------------------- | ------------------------------------------------------------- |
| 宿主机 | Ubuntu x86_64                | 22.04                     | 24.04 已实测可用；仅需驱动 + Docker                           |
| 显卡   | NVIDIA GPU，显存             | ≥ 8 GB                   | RTX 4060 8 GB 为实测下限                                      |
| 显卡   | NVIDIA 驱动                  | ≥ 535.129.03 且 ≤ 580.x | **595.x 会导致 RTX 渲染器段错误**，实测 580.178.04 可用 |
| 容器   | Docker Engine                | ≥ 24.0                   | 实测 29.10.0                                                  |
| 容器   | Docker Compose v2            | ≥ 2.0.0                  | 使用 `docker compose` 子命令                                |
| 容器   | nvidia-container-toolkit     | 最新稳定版                | Docker 28+ 必须生成 CDI 规格                                  |
| 磁盘   | `/var/lib/docker` 可用空间 | ≥ 80 GB                  | 首次拉取 + 解压 + 构建缓存峰值约 32 GB，留余量                |
| 容器内 | Ubuntu                       | 22.04                     | 由镜像固定                                                    |
| 容器内 | ROS 2                        | Humble                    | 镜像内预装                                                    |
| 容器内 | Python                       | 3.10                      | Isaac Sim 与 Humble 的 ABI 基线                               |
| 容器内 | colcon / xacro / cmake       | Humble 配套               | 镜像内已装                                                    |
| 容器内 | Isaac Sim                    | 4.5.0                     | 官方镜像 `nvcr.io/nvidia/isaac-sim:4.5.0`                   |
| 可视化 | X11 桌面或浏览器             | Chrome / Edge             | `livestream` 模式不依赖 X11，推荐                           |

### 2.2 一键检查本机版本

```bash
# 宿主机系统
lsb_release -ds

# 显卡与驱动（驱动主版本必须在 535~580 之间）
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

# Docker 与 Compose
docker --version && docker compose version

# NVIDIA 容器工具链
nvidia-ctk --version

# 磁盘余量
df -h /var/lib/docker
```

> 版本不满足时不要继续往下走。尤其是驱动 595.x 与显存不足，会在 Isaac Sim 启动阶段直接失败。

---

## 3. 环境配置（分步执行）

以下命令均在仓库根目录 `/home/moke/bajie_ws` 下执行。

### 3.1 第一步：宿主机体检（只读，不改系统）

```bash
bash scripts/host_check.sh
```

检查 GPU、驱动版本、GPU 穿透、磁盘和 X11。输出末尾应为：

```text
结果：N 通过 / 0 阻塞
```

也可以使用带 `--check` 的完整脚本做干跑（只检查、不安装）：

```bash
sudo bash scripts/setup_host.sh --check
```

### 3.2 第二步：安装 NVIDIA Container Toolkit 并注册 CDI

`setup_host.sh` 是幂等的：每步先检测，已满足即跳过；遇到错误立即停止；不会删除任何文件。

```bash
sudo bash scripts/setup_host.sh
```

脚本依次完成：检查 GPU/驱动 → 安装 `nvidia-container-toolkit` → 注册 Docker runtime →
生成 `/etc/cdi/nvidia.yaml` → 验证容器 GPU → 检查磁盘 → 拉取基础镜像 → 构建 `bajie/sim:humble45` → 冒烟测试。

如果只需要手工补装工具链：

```bash
# 安装
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# 注册 nvidia runtime（会重启 Docker，正在运行的容器会中断）
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Docker 28+ 的 --gpus 走 CDI，必须生成规格
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# 验收：容器内能看到显卡
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

### 3.3 第三步：构建镜像

```bash
# 仿真一体化镜像（Isaac Sim 4.5 + ROS 2 Humble）
docker compose -f docker/compose.yaml build

# 可选：RViz 可视化镜像（在上一镜像基础上补装 rviz2，约 +1.2 GB）
docker build -f docker/Dockerfile.rviz -t bajie/sim:humble45-rviz docker
```

首次构建需要从 `nvcr.io` 拉取约 7.3 GB 基础镜像，耗时较长，建议放到后台执行。

### 3.4 第四步：初始化工作区（编译 + 展开 URDF）

```bash
docker compose -f docker/compose.yaml run --rm sim bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /workspace &&
  colcon build --base-paths src --symlink-install &&
  bash /workspace/scripts/expand_urdf.sh
'
```

产物：

- `build/ install/ log/`：colcon 编译输出
- `generated/bajie_robot.urdf`：官方 xacro 展开后的 URDF（`package://` 已替换为绝对路径）

### 3.5 第五步：确认环境变量

| 变量                                  | 推荐值               | 作用                                               |
| ------------------------------------- | -------------------- | -------------------------------------------------- |
| `ROS_DOMAIN_ID`                     | `42`               | ROS 2 域号。仿真容器、RViz 容器必须一致            |
| `RMW_IMPLEMENTATION`                | `rmw_fastrtps_cpp` | 镜像内已预编译；跨容器通信配合 `--ipc host` 使用 |
| `DISPLAY`                           | `:0` / `:1`      | X11 显示号，`gui` 模式需要                       |
| `OMNI_KIT_ALLOW_ROOT`               | `1`                | 容器内以 root 运行 Kit 的放行开关                  |
| `ACCEPT_EULA` / `PRIVACY_CONSENT` | `Y`                | Isaac Sim 非交互启动许可                           |
| `NVIDIA_DRIVER_CAPABILITIES`        | `all`              | 让容器能使用 OpenGL/Vulkan                         |
| `__GL_THREADED_OPTIMIZATIONS`       | `0`                | 修复 RViz 3D 区域黑屏（已固化在 rviz 镜像内）      |

`run_sim.sh` 与 `docker/compose.yaml` 已自动带上上述大部分变量，手工 `docker run` 时需要自己补。

---

## 4. 启动方式

### 4.1 开发模式：进入容器交互开发

适合改代码、编译、临时执行 ROS 命令。

```bash
docker compose -f docker/compose.yaml run --rm sim
```

进入容器后：

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --base-paths src --symlink-install
source /workspace/install/setup.bash   # 注意：不能依赖 /etc/profile.d 的默认路径
```

> 镜像内 `/etc/profile.d/99-ros-humble.sh` 默认 source 的是 `/workspace/bajie_ws/install`，
> 而仓库实际挂在 `/workspace`，所以每次进入容器都要显式 source `/workspace/install/setup.bash`。

### 4.2 生产模式：镜像构建 + 固定参数启动

适合复现、批量回归和交付，不依赖交互 shell。

```bash
# 1) 构建（只需一次；镜像变更后重跑）
docker build -f docker/Dockerfile.sim -t bajie/sim:humble45 docker

# 2) 启动并执行固定命令
docker run --rm --gpus all --network host --ipc host --shm-size=2gb \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e ROS_DOMAIN_ID=42 -e OMP_NUM_THREADS=4 \
  -v $PWD:/workspace \
  -v $PWD/.ov-cache:/root/.cache/ov \
  -v $PWD/.ov-data:/root/.local/share/ov \
  bajie/sim:humble45 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
            source /workspace/install/setup.bash && \
            /isaac-sim/python.sh /workspace/scripts/smoke_sim.py'
```

关键参数：

| 参数                                | 必要性   | 说明                                                  |
| ----------------------------------- | -------- | ----------------------------------------------------- |
| `--gpus all`                      | 必须     | 不传会在 RTX 初始化处崩溃                             |
| `--network host`                  | 必须     | DDS 发现、WebRTC 推流、ROS 2 通信都依赖               |
| `--ipc host` + `--shm-size=2gb` | 必须     | 避免 FastDDS 跨容器共享内存导致“能 list 收不到数据” |
| `-v $PWD:/workspace`              | 必须     | USD 内的相对资源路径按 `/workspace` 解析            |
| `.ov-cache` / `.ov-data`        | 强烈建议 | 不挂会每次重编着色器，冷启动约 4 分钟                 |
| `-e OMNI_KIT_ALLOW_ROOT=1`        | GUI 必须 | 缺少会在加载 experience 前直接退出                    |

### 4.3 仿真三种模式

统一入口 `scripts/run_sim.sh`，默认场景为 `/workspace/src/bajie_sim/scene/abr_scene.usd`。

```bash
# GUI：有头窗口，需要 X11
bash scripts/run_sim.sh gui

# WebRTC 推流：无头运行，显存更省（推荐）
bash scripts/run_sim.sh livestream

# 纯无头：直接跑指定脚本
bash scripts/run_sim.sh headless -- /workspace/scripts/smoke_sim.py
```

启动后可访问：

| 模式           | 访问方式                                                       | 端口                                     |
| -------------- | -------------------------------------------------------------- | ---------------------------------------- |
| `livestream` | 浏览器打开 `https://<宿主机IP>:8211/streaming/webrtc-client` | 8211（Web 客户端），49100（WebRTC 信令） |
| `gui`        | 宿主机 X11 窗口                                                | 无                                       |

### 4.4 RViz 可视化

```bash
# 构建一次
docker build -f docker/Dockerfile.rviz -t bajie/sim:humble45-rviz docker

# 启动
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
  bash -lc 'source /opt/ros/humble/setup.bash && \
            source /workspace/install/setup.bash && \
            ros2 launch bajie_robot display_bajie_robot_urdf.launch.py use_sim_time:=false'
```

单独看模型时没有 `/clock`，必须 `use_sim_time:=false`，否则 TF 时间戳异常。

### 4.5 题一任务启动

题一比赛动作使用 `bajie_tasks` 的 launch，而不是上面只看模型的官方命令：

```bash
docker run --rm -it --name bajie-q1 \
  --network host --ipc host --shm-size=2gb \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:ro" \
  -e XAUTHORITY=/root/.Xauthority \
  -v $PWD:/workspace -w /workspace \
  bajie/sim:humble45-rviz \
  bash -lc 'source /opt/ros/humble/setup.bash && \
            source /workspace/install/setup.bash && \
            ros2 launch bajie_tasks q1_rviz.launch.py \
              input_file:=/workspace/inputs/q1_demo_input.jsonl \
              output_file:=/workspace/answers/q1_demo_output.jsonl'
```

### 4.6 常见启动参数

| 参数              | 适用脚本              | 默认值                                           | 说明             |
| ----------------- | --------------------- | ------------------------------------------------ | ---------------- |
| `SCENE`         | `run_sim.sh`        | `/workspace/src/bajie_sim/scene/abr_scene.usd` | 覆盖场景文件     |
| `IMAGE`         | `run_sim.sh`        | `bajie/sim:humble45`                           | 覆盖镜像         |
| `NAME`          | `run_sim.sh`        | `bajie-sim`                                    | 容器名           |
| `ROS_DOMAIN_ID` | 全部                  | `42`                                           | 多端通信必须一致 |
| `--scene`       | `run_scene_ros.py`  | 官方场景                                         | 场景路径         |
| `--duration`    | `run_scene_ros.py`  | `10`                                           | 播放的仿真秒数   |
| `--report`      | `run_scene_ros.py`  | `/workspace/logs/ros_live_report.txt`          | 报告输出路径     |
| `input_file`    | `q1_rviz.launch.py` | `/workspace/inputs/q1_demo_input.jsonl`        | 题一输入         |
| `output_file`   | `q1_rviz.launch.py` | `/workspace/answers/q1_demo_output.jsonl`      | 题一作答         |

---

## 5. 运行验证

### 5.1 宿主机层

```bash
bash scripts/host_check.sh
```

预期末尾：`结果：N 通过 / 0 阻塞`。

```bash
sudo bash scripts/setup_host.sh --check
```

预期末尾：`体检完成（--check 模式，未做任何修改）`。

### 5.2 容器基础环境

```bash
docker compose -f docker/compose.yaml run --rm sim bash -lc '
  source /opt/ros/humble/setup.bash
  echo "ROS_DISTRO=$ROS_DISTRO"
  ros2 --version
  xacro --version
  python3 --version
'
```

预期输出：`ROS_DISTRO=humble`、`ros2` 与 `xacro` 均可打印版本、Python 为 `3.10.x`。

ROS 2 Bridge 自检（无输出且退出码 0 即通过）：

```bash
docker run --rm bajie/sim:humble45 bash -lc '
  export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
  /isaac-sim/exts/isaacsim.ros2.bridge/bin/isaacsim.ros2.bridge.check; echo "check exit=$?"'
```

### 5.3 场景与桥接数据面

场景结构体检（结果以文件为准，Isaac Sim 会重定向 stdout）：

```bash
docker run --rm --gpus all --network host --ipc host --shm-size=2gb \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v $PWD:/workspace -v $PWD/.ov-cache:/root/.cache/ov -v $PWD/.ov-data:/root/.local/share/ov \
  bajie/sim:humble45 bash -lc 'source /opt/ros/humble/setup.bash && \
    /isaac-sim/python.sh /workspace/scripts/smoke_sim.py'
```

预期 `logs/smoke_report.txt` 中包含：1 个 Articulation Root、34 个关节、4 个相机 + 1 个 Lidar。

ROS 2 桥接实测（官方场景本身不含桥接图，需要现场搭最小图）：

```bash
docker run --rm --gpus all --network host --ipc host --shm-size=2gb \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v $PWD:/workspace -v $PWD/.ov-cache:/root/.cache/ov -v $PWD/.ov-data:/root/.local/share/ov \
  bajie/sim:humble45 bash -lc 'source /opt/ros/humble/setup.bash && \
    /isaac-sim/python.sh /workspace/scripts/build_bridge.py'
```

预期 `logs/bridge_report.txt` 中 `/clock` 与 `/joint_states` 均有持续计数（实测 25 秒约 183 / 182 条）。

时间轴 + 话题实测：

```bash
bash scripts/run_sim.sh headless -- /workspace/scripts/run_scene_ros.py --duration 8
```

预期生成 `logs/ros_live_report.txt`，报告内包含话题接收条数。

### 5.4 可视化与任务层

| 检查项          | 方法                                                     | 预期                                        |
| --------------- | -------------------------------------------------------- | ------------------------------------------- |
| livestream 页面 | 浏览器打开 `https://<IP>:8211/streaming/webrtc-client` | 出现 WebRTC 连接页并可进入画面              |
| RViz 模型       | `ros2 launch bajie_robot ...`                          | 显示整机模型，TF 树无报错                   |
| ROS 话题        | 容器内 `ros2 topic list`                               | 出现 `/clock`、`/joint_states`、`/tf` |
| 话题频率        | `ros2 topic hz /joint_states`                          | 有稳定频率输出，不停在等待                  |
| 题一作答        | `answers/q1_demo_output.jsonl`                         | 一行结果，含 5 个按序关键帧及最终状态       |

```bash
# 容器内确认话题
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /joint_states
```

---

## 6. 常见运行问题排查

### 6.1 端口占用

`livestream` 依赖 8211 / 49100；`--network host` 模式下端口冲突会直接导致启动失败或页面打不开。

```bash
# 查看端口占用
ss -ltnp | grep -E '8211|49100'

# 查看残留容器
docker ps -a --format '{{.Names}}\t{{.Status}}\t{{.Ports}}' | grep -E 'bajie|isaac'
```

处理：

```bash
docker stop bajie-sim          # 停掉旧仿真容器
# 若非容器进程占用，按 ss 输出中的 PID 处理；无法停时更换端口后重启
```

### 6.2 依赖缺失

| 现象                                                       | 原因                             | 处理                                                                                 |
| ---------------------------------------------------------- | -------------------------------- | ------------------------------------------------------------------------------------ |
| `xacro: command not found`                               | 镜像不对或未装 xacro             | 使用 `bajie/sim:humble45`，或 `apt install ros-humble-xacro`                     |
| `executable 'rviz2' not found`                           | 仿真镜像默认未装 rviz2           | 改用 `bajie/sim:humble45-rviz`                                                     |
| `librmw_fastrtps_cpp.so: cannot open shared object file` | 启动前未 source Humble           | 先 `source /opt/ros/humble/setup.bash`，再启动节点                                 |
| Python 报 `ModuleNotFoundError: rclpy`                   | 用了系统 Python 而非镜像 Python  | 用 `/isaac-sim/python.sh` 运行脚本                                                 |
| `held broken packages`（构建 rviz 镜像）                 | 基础镜像 `libbrotli1` 版本冲突 | 使用 `Dockerfile.rviz` 中已固化的 `--allow-downgrades` 方案重建                  |
| `unrecognized type 'isaacsim.ros2.bridge.ROS2Context'`   | 未启用 ROS 2 Bridge 扩展         | 脚本内先 `set_extension_enabled_immediate("isaacsim.ros2.bridge", True)`           |
| 工作区包找不到                                             | 未编译或未 source                | `colcon build --base-paths src --symlink-install` 后 source `install/setup.bash` |

### 6.3 权限错误

| 现象                                             | 原因                                           | 处理                                                          |
| ------------------------------------------------ | ---------------------------------------------- | ------------------------------------------------------------- |
| `permission denied ... /var/run/docker.sock`   | 当前用户不在 docker 组                         | `sudo usermod -aG docker $USER` 后重新登录                  |
| `build/ install/ log/` 无法写入或属主为 root   | 容器以 root 编译                               | `sudo chown -R $USER:$USER build install log`               |
| `XIO: fatal IO error ...` 或 X 认证失败        | `~/.Xauthority` 不存在，被 Docker 挂成空目录 | `rmdir ~/.Xauthority`；`run_sim.sh` 会自动探测真实 cookie |
| `Omniverse Kit cannot be run as the root user` | 缺少 root 放行                                 | 加 `-e OMNI_KIT_ALLOW_ROOT=1`（`run_sim.sh` 已带）        |
| 挂载目录无权限                                   | 宿主目录属主与容器 UID 不一致                  | 调整目录属主，或用 `--user $(id -u):$(id -g)` 启动          |

### 6.4 GPU 与驱动

| 现象                                                               | 原因                                                       | 处理                                                                                                                              |
| ------------------------------------------------------------------ | ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `no known GPU vendor found` / `could not select device driver` | Docker 未注册 nvidia runtime 或缺 CDI 规格                 | `sudo nvidia-ctk runtime configure --runtime=docker && sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml`，重启 Docker |
| 启动后段错误，崩在 `librtx.scenedb.plugin.so`                    | 驱动 595.x 未经 Isaac Sim 4.5 验证                         | 降级到 580 或 550 系列并重启                                                                                                      |
| 启动即 OOM                                                         | 8 GB 显存 + 高分辨率 + 光追                                | 用 `livestream`/`--no-window`，降低分辨率，关闭 path tracing                                                                  |
| `nvidia-smi` 在容器内不可用                                      | 未加 `--gpus all` 或 `NVIDIA_DRIVER_CAPABILITIES` 不全 | 补 `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all`                                                                               |

### 6.5 显示与渲染

| 现象                                            | 原因                                        | 处理                                                      |
| ----------------------------------------------- | ------------------------------------------- | --------------------------------------------------------- |
| RViz UI 正常但 3D 区域纯黑                      | GL 线程优化与 Ogre GLX 冲突                 | 确认 `__GL_THREADED_OPTIMIZATIONS=0`（rviz 镜像已固化） |
| GUI 窗口打开但场景为空，日志只有 `World0.usd` | usd 路径作为位置参数不会触发 `open_stage` | 用 `scripts/run_gui.py` 启动                            |
| 模型纯白/灰色                                   | 官方 mesh 无贴图                            | 正常现象                                                  |
| Wayland 下 GUI 卡在 GL 初始化                   | X11 兼容性问题                              | 改用 `livestream` 模式                                  |

### 6.6 ROS 2 通信

| 现象                           | 原因                                            | 处理                                                        |
| ------------------------------ | ----------------------------------------------- | ----------------------------------------------------------- |
| `ros2 topic list` 为空       | 未搭桥接图、`ROS_DOMAIN_ID` 不一致、未 source | 用 `build_bridge.py` 建图；两端统一 domain；先 source     |
| 话题能 list 但 `hz` 一直等待 | FastDDS 跨容器共享内存不通                      | `--ipc host --shm-size=2gb`，并统一 RMW                   |
| TF 报 duplicate                | 场景已有 PublishTransformTree，又重复搭了一个   | 删除自己额外添加的节点                                      |
| 时间戳跳变、物理抖动           | 双发 `/clock`                                 | 只保留一套 Clock / JointState 发布                          |
| 桥接图挂上但一帧数据不发       | 官方图根是 `OmniGraph` 而非 `ActionGraph`   | 不要挂官方图，改用 `scripts/build_bridge.py` 现场搭最小图 |

---

## 7. 当前状态与已知限制

| 项           | 状态（2026-11-20 实测）                                                          |
| ------------ | -------------------------------------------------------------------------------- |
| 驱动         | 580.178.04（必须 ≤580；595.x 会让 RTX 渲染器在 `librtx.scenedb` 段错误）      |
| 镜像         | `bajie/sim:humble45`，含 Humble + `rmw_fastrtps_cpp` + ROS 2 Bridge 预编译库 |
| GPU 穿透     | 容器内 `nvidia-smi` 正常                                                       |
| 官方场景加载 | `abr_scene.usd` 可打开，1 个 Articulation Root / 34 关节 / 4 相机 + 1 Lidar    |
| 时间轴       | 脚本 `play` 生效（仿真时间实测推进 0 → 87s）                                  |
| ROS 桥接     | 官方场景内无 ROS 2 图节点，需自己搭最小桥接图                                    |
| RViz2        | `bajie/sim:humble45-rviz` 可用，黑屏问题已修复并固化                           |

官方桥接图（`machine/component/omni_graphs/*.usd`）挂载后不发布数据，已实测确认；
可靠路径是 `scripts/build_bridge.py` 用 `og.Controller` 现场搭图。T1 的稳定跑法为单进程一体化：
开场景 → 建图 → 播放 → 驱动 → 记录。

12 个可控关节（实测 `/joint_states`）：

```text
底盘   wheel_left_joint  wheel_right_joint
升降   lift_joint
头部   pitch_joint
机械臂 joint1 … joint6
夹爪   arm_finger_joint1  arm_finger_joint2
```

> URDF 里轮子是 `fixed`（静态 TF 用），但场景 USD 里是 `PhysicsRevoluteJoint`，
> 因此仿真中底盘可以移动，这一点以 USD 为准。

更多环境细节与踩坑记录见 [docs/开发环境搭建.md](docs/开发环境搭建.md)。

---

## 8. 提交合规

- 模型必须基于官方 URDF：`generated/bajie_robot.urdf` 由官方 xacro 直接展开，未改几何；
- 导入 Isaac Sim 时**禁止勾选 Merge Fixed Joints**（31 个关节里有 21 个 fixed，勾了就等于改了模型）；
- 用 `scripts/check_urdf_origin.py` 逐条比对 origin，输出可直接作为合规证据；
- `logs/smoke_report.txt`（场景结构）与 `logs/diag_graphs.txt`（桥接状态）同样可作为证据材料；
- 正式提交只将 `bajie-demo/` 放到独立 AtomGit 作品仓库根目录，并改为报名姓名。
