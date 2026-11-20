#!/usr/bin/env bash
# 启动仿真。三种模式：
#   bash scripts/run_sim.sh gui               # 有头 GUI（X11 桌面直接看画面）
#   bash scripts/run_sim.sh livestream        # 无头 + WebRTC 推流（显存更省）
#   bash scripts/run_sim.sh headless -- test  # 纯无头跑脚本（CI / 批量回归）
#
# 场景路径可用 SCENE 覆盖：SCENE=/workspace/src/bajie_sim/scene/abr_scene.usd bash scripts/run_sim.sh gui
set -euo pipefail

MODE="${1:-gui}"
SCENE="${SCENE:-/workspace/src/bajie_sim/scene/abr_scene.usd}"
IMAGE="${IMAGE:-bajie/sim:humble45}"
NAME="${NAME:-bajie-sim}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# 本机 NVIDIA 驱动需要能画 OpenGL/Vulkan；graphics 能力缺了会在 RTX 初始化处崩
GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all)

# --- X 认证探测 ---------------------------------------------------------------
# 坑：GDM 会话下 ${HOME}/.Xauthority 往往根本不存在，而 docker -v 对缺失的源路径
# 会「自动创建一个空目录」挂进去。容器内 /root/.Xauthority 成了目录，X 认证失效，
# Isaac Sim 建窗口那一刻报 "XIO: fatal IO error on X server \":1\"" 然后退出码 1。
# 所以必须找到真实 cookie 文件（文件，不是目录），找不到就不挂、退回 xhost 授权。
detect_xauth() {
  local uid cand
  uid="$(id -u)"
  for cand in "${XAUTHORITY:-}" "${HOME}/.Xauthority" \
              "/run/user/${uid}/gdm/Xauthority" \
              /run/user/"${uid}"/.mutter-Xwaylandauth.*; do
    if [ -n "$cand" ] && [ -f "$cand" ]; then printf '%s' "$cand"; return 0; fi
  done
  return 1
}

COMMON=(
  --rm
  --name "$NAME"
  --network host
  --ipc host
  --shm-size=2gb
  -e DISPLAY="${DISPLAY:-:1}"
  -e QT_X11_NO_MITSHM=1
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  # 容器内是 root，Kit 默认拒绝启动；缺这个会在加载 experience 前直接退出
  -e OMNI_KIT_ALLOW_ROOT=1
  -e OMP_NUM_THREADS=4
  -e ACCEPT_EULA=Y
  -e PRIVACY_CONSENT=Y
  -v "$ROOT":/workspace
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  # 着色器/扩展缓存：不挂的话每次启动都要重编约 4 分钟
  -v "$ROOT/.ov-cache:/root/.cache/ov"
  -v "$ROOT/.ov-data:/root/.local/share/ov"
)

# --rm 容器的可写层会随退出丢弃；RTX 的 PSO/驱动着色器缓存在
# /isaac-sim/kit/cache 下，不持久化的话每次重启都要重新编译（首帧可达数分钟）。
mkdir -p "$ROOT/.kit-cache" "$ROOT/.kit-data"
COMMON+=(-v "$ROOT/.kit-cache:/isaac-sim/kit/cache"
         -v "$ROOT/.kit-data:/isaac-sim/kit/data")

# 把 Kit 日志挂出来：--rm 会让容器一退出日志就消失，
# 出问题（比如 "There was an error running python"）时就没 traceback 可查了。
mkdir -p "$ROOT/logs/kit"
COMMON+=(-v "$ROOT/logs/kit:/isaac-sim/kit/logs")

# 注意：不要在这里预设 RMW_IMPLEMENTATION。镜像里 RMW 库在 /opt/ros/humble 下，
# 没 source 就设了它，rcl 会因 dlopen 失败让 ROS2 Bridge 扩展整个启动失败。
if XAUTH="$(detect_xauth)"; then
  COMMON+=(-v "$XAUTH:/root/.Xauthority:ro" -e XAUTHORITY=/root/.Xauthority)
  echo "[run_sim] X 认证：$XAUTH"
else
  echo "[run_sim] 未找到 Xauthority 文件，依赖 xhost 授权" >&2
fi

# 所有模式都在 source 过 Humble 的环境里启动：ROS2 Bridge 扩展的 RMW/消息类型
# 依赖 AMENT_PREFIX_PATH 与 LD_LIBRARY_PATH，裸启动会报
# "librmw_fastrtps_cpp.so: cannot open shared object file"。
ROS_ENV='source /opt/ros/humble/setup.bash && exec'

case "$MODE" in
  gui)
    command -v xhost >/dev/null && xhost +si:localuser:root >/dev/null 2>&1 || true
    # 走 run_gui.py 而不是 isaac-sim.sh + usd 路径：后者不会 open_stage，
    # 只会停在默认的 World0 空场景，且不播放时间轴。
    exec docker run -it "${GPU_ARGS[@]}" "${COMMON[@]}" "$IMAGE" \
      bash -lc "$ROS_ENV /isaac-sim/python.sh /workspace/scripts/run_gui.py --scene '$SCENE'"
    ;;
  livestream)
    # 信令端口见容器内 WebRTC 配置；无浏览器客户端时用 run_scene_ros.py 走纯无头
    exec docker run -it "${GPU_ARGS[@]}" "${COMMON[@]}" "$IMAGE" \
      bash -lc "$ROS_ENV /isaac-sim/isaac-sim.streaming.sh --allow-root --no-window '$SCENE'"
    ;;
  headless)
    shift
    [ "${1:-}" = "--" ] && shift
    TARGET=("${@:-/workspace/scripts/smoke_sim.py}")
    exec docker run "${GPU_ARGS[@]}" "${COMMON[@]}" "$IMAGE" \
      bash -lc "$ROS_ENV /isaac-sim/python.sh ${TARGET[*]}"
    ;;
  *)
    echo "未知模式：$MODE（gui | livestream | headless）" >&2; exit 2 ;;
esac
