#!/usr/bin/env bash
# 一次性把宿主机配成「能跑 Isaac Sim 4.5 容器」的状态。
#
# 在真实终端里运行（需要 root）：
#     sudo bash scripts/setup_host.sh
#     sudo bash scripts/setup_host.sh --check      # 只体检，不改任何东西
#
# 设计原则：
#   * 幂等——每一步先检测，已满足就跳过；重复运行安全
#   * 遇错即停——不"继续尝试"，避免半成品状态难以诊断
#   * 不删任何东西——磁盘不足只给清单和建议，删什么由你决定
set -Eeuo pipefail

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; NC=$'\033[0m'
step() { echo; echo "${GRN}==> $*${NC}"; }
warn() { echo "${YEL}[WARN] $*${NC}"; }
die()  { echo "${RED}[FAIL] $*${NC}" >&2; exit 1; }

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_TAG="bajie/sim:humble45"
BASE_IMAGE="nvcr.io/nvidia/isaac-sim:4.5.0"

step "0. 身份与前提"
[ "$(id -u)" -eq 0 ] || die "需要 root：sudo bash scripts/setup_host.sh"
echo "  仓库：$ROOT"
echo "  用户：$(id -un)"

step "1. GPU 与驱动"
command -v nvidia-smi >/dev/null || die "没有 nvidia-smi：先装 NVIDIA 驱动（>=535.129.03）"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | sed 's/^/  /'
DRV_MAJOR=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1 | cut -d. -f1)
[ "$DRV_MAJOR" -ge 535 ] || die "驱动 $DRV_MAJOR 过旧，Isaac Sim 4.5 要求 >= 535.129.03"
VRAM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -1)
[ "$VRAM" -ge 7900 ] || warn "显存 ${VRAM}MiB 低于 8G，场景复杂时会在 RTX 初始化处失败"

step "2. NVIDIA Container Toolkit"
if command -v nvidia-ctk >/dev/null 2>&1; then
  echo "  已安装：$(nvidia-ctk --version | head -1)"
else
  if [ "$CHECK_ONLY" -eq 1 ]; then warn "未安装（--check 模式不安装）"; else
    echo "  安装中…"
    # 首选 NVIDIA 官方仓库；nvidia.github.io 不可达时退回 Ubuntu 自带源（noble universe 里有）
    if curl -fsSL --max-time 15 https://nvidia.github.io/libnvidia-container/gpgkey >/dev/null 2>&1; then
      install -m 0755 -d /usr/share/keyrings
      curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
      curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        > /etc/apt/sources.list.d/nvidia-container-toolkit.list
      apt-get update -qq
    else
      warn "nvidia.github.io 不可达，改用 Ubuntu 官方源安装"
    fi
    apt-get install -y -qq nvidia-container-toolkit \
      || die "toolkit 安装失败：检查 apt 源，或手动 apt install nvidia-container-toolkit"
    echo "  完成：$(nvidia-ctk --version | head -1)"
  fi
fi

step "3. 注册 nvidia runtime 并生成 CDI 规格"
if docker info 2>/dev/null | grep -qi nvidia; then
  echo "  nvidia runtime 已注册，跳过"
elif [ "$CHECK_ONLY" -eq 1 ]; then
  warn "未注册（--check 模式不修改）"
else
  nvidia-ctk runtime configure --runtime=docker
  warn "即将重启 docker，正在运行的容器会被中断（含你现有的 ubuntu22 容器）"
  systemctl restart docker
  for _ in $(seq 1 20); do docker info >/dev/null 2>&1 && break; sleep 1; done
fi

# 关键：Docker 28+ 的 --gpus 走 CDI 设备发现，runtime configure 只注册 runtime、
# 不生成规格。没有 /etc/cdi/nvidia.yaml 就会报 "no known GPU vendor from CDI"。
if [ "$CHECK_ONLY" -eq 1 ]; then
  if [ ! -f /etc/cdi/nvidia.yaml ] && [ ! -f /var/run/cdi/nvidia.yaml ]; then
    warn "CDI 规格缺失（--check 模式不生成）——这是 --gpus 报 no known GPU vendor 的常见根因"
  fi
elif [ ! -f /etc/cdi/nvidia.yaml ] && [ ! -f /var/run/cdi/nvidia.yaml ]; then
  echo "  生成 CDI 规格…"
  nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml || \
    warn "CDI 生成失败——不影响 runtime 路径，但第 4 步大概率不过"
fi

step "4. 验证容器能拿到 GPU"
# 用本地已有镜像探测设备节点，不现拉大镜像（nvidia/cuda 那 200MB 曾让 --check 卡在下载上）
GPU_OUT=$(timeout 60 docker run --rm --gpus all ubuntu:22.04 sh -c 'ls /dev/nvidia* 2>/dev/null' 2>&1) || true
if echo "$GPU_OUT" | grep -q "/dev/nvidia"; then
  echo "  ${GRN}GPU 穿透 OK${NC}：$(echo "$GPU_OUT" | tr '\n' ' ')"
elif [ "$CHECK_ONLY" -eq 1 ]; then
  echo "$GPU_OUT" | sed 's/^/  /'
  warn "GPU 穿透未通过——--check 模式不装 toolkit，属预期；跑不带 --check 的版本即可"
else
  echo "$GPU_OUT" | sed 's/^/  /'
  echo "  --- 自动诊断 ---"
  command -v nvidia-ctk >/dev/null && echo "  nvidia-ctk: $(nvidia-ctk --version | head -1)" \
    || echo "  nvidia-ctk: 未安装（toolkit 没装成，回看第 2 步输出）"
  ls /etc/cdi /var/run/cdi 2>/dev/null | sed 's/^/  cdi: /'
  docker info 2>/dev/null | grep -i runtimes | sed 's/^/  /'
  die "GPU 穿透失败。按上方线索排查；常见修法：sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml && sudo systemctl restart docker"
fi

step "5. 磁盘预算"
AVAIL=$(df -BG --output=avail /var/lib/docker 2>/dev/null | tail -1 | tr -dc '0-9')
AVAIL=${AVAIL:-0}
echo "  /var/lib/docker 剩余：${AVAIL}G"

# 需求随状态变：镜像已在本地时，只剩运行期缓存（shader cache 2-4G、日志、checkpoint）
if docker image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
  NEED=8; WHY="镜像已在本地，只需覆盖运行缓存（shader cache/日志，约 5-8G）"
else
  NEED=32; WHY="Isaac Sim 压缩 7.3G + 解压约 22G + Humble 约 2G ≈ 32G 峰值"
fi
echo "  当前需求：${NEED}G（$WHY）"

if [ "$AVAIL" -lt "$NEED" ]; then
  warn "空间不足。以下是可清理项（本脚本不会替你删，删什么你定）："
  du -sh "$HOME"/.cache/* 2>/dev/null | sort -h | tail -8 | sed 's/^/    /'
  echo "    另可考虑：docker builder prune ｜ ~/.npm 缓存 ｜ 系统回收站"
  die "先腾出至少 ${NEED}G 再重跑本脚本"
elif [ "$AVAIL" -lt $((NEED * 2)) ]; then
  warn "${AVAIL}G 够用但余量不大——Isaac Sim 首次启动 shader cache 会吃 2-4G，留意后续下载"
else
  echo "  ${GRN}磁盘余量 ${AVAIL}G，需求 ${NEED}G${NC}"
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
  step "体检完成（--check 模式，未做任何修改）"; exit 0
fi

step "6. 拉取官方镜像（7.3G，慢，建议挂后台）"
if docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
  echo "  已存在，跳过"
else
  docker pull "$BASE_IMAGE"
fi

step "7. 构建 bajie/sim:humble45"
cd "$ROOT"
docker build -f docker/Dockerfile.sim -t "$IMAGE_TAG" docker/

step "8. 冒烟"
docker run --rm --gpus all "$IMAGE_TAG" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  echo "  ROS_DISTRO=$ROS_DISTRO"
  echo "  ros2:   $(ros2 --version 2>/dev/null | head -1)"
  echo "  xacro:  $(xacro --version 2>&1 | head -1)"
  echo "  colcon: $(colcon version-check 2>/dev/null | head -1 || echo ok)"
  echo "  python: $(python3 --version)"
  # 实测路径：bridge 主库在 bin/，不在 humble/lib/（humble/lib 里是它依赖的一套 ROS 运行库）
  if ls /isaac-sim/exts/isaacsim.ros2.bridge/bin/libisaacsim.ros2.bridge.humble.so >/dev/null 2>&1; then
    echo "  bridge: 主库就位"
    LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
      /isaac-sim/exts/isaacsim.ros2.bridge/bin/isaacsim.ros2.bridge.check \
      && echo "  bridge: 自检通过（无缺失符号）"
  else
    echo "  bridge: 未找到主库，需跑 /isaac-sim/build_ros2_workspace.sh 重编"
  fi
'

step "完成"
cat <<EOF
  下一步：
    docker compose -f docker/compose.yaml run --rm sim            # 进容器
    bash scripts/run_sim.sh livestream                           # 起仿真（浏览器开 :8211）
    /isaac-sim/python.sh /workspace/scripts/smoke_sim.py         # 场景体检
EOF
