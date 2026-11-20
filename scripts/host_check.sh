#!/usr/bin/env bash
# 开发前的硬闸门体检。任何一个 FAIL 都不要往下走，先修它。
# 用法：bash scripts/host_check.sh
set -uo pipefail

PASS=0; FAIL=0
ok()   { echo -e "  [ OK ] $*"; PASS=$((PASS+1)); }
bad()  { echo -e "  [FAIL] $*"; FAIL=$((FAIL+1)); }
warn() { echo -e "  [WARN] $*"; }

echo "== 1. GPU 与驱动 =="
if nvidia-smi >/dev/null 2>&1; then
  ok "nvidia-smi 可用"
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | sed 's/^/         /'
  DRV=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1 | cut -d. -f1)
  VRAM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -1)
  if [ "${DRV:-0}" -ge 535 ]; then ok "驱动 ${DRV} >= 535（Isaac Sim 4.5 要求 >=535.129.03）"
  else bad "驱动 ${DRV} 过旧，需 >= 535.129.03"; fi
  if [ "${VRAM:-0}" -ge 7900 ]; then warn "显存 ${VRAM}MiB：贴着下限，abr_scene 复杂时要降分辨率/关光追"
  else bad "显存 ${VRAM}MiB 低于 8G 下限，Isaac Sim 4.5 大概率起不来"; fi
else
  bad "nvidia-smi 不可用：驱动没装好，或被当前 shell 的沙箱屏蔽（换普通终端再试）"
fi

echo "== 2. NVIDIA Container Toolkit（容器能不能看到 GPU）=="
if command -v nvidia-ctk >/dev/null 2>&1; then
  ok "nvidia-ctk 已安装"
  if docker info 2>/dev/null | grep -qi nvidia; then ok "docker 已注册 nvidia runtime"
  else bad "已装 toolkit 但 docker 未注册，跑：sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"; fi
else
  bad "未安装 nvidia-container-toolkit —— 这是 Isaac Sim 容器化的第一道闸门"
fi
if docker run --rm --gpus all hello-world >/tmp/.gpu_test 2>&1; then
  ok "容器内可调用 GPU（--gpus all 实测通过）"
else
  if grep -qi "no known GPU vendor\|could not select device driver" /tmp/.gpu_test; then
    bad "容器内看不到 GPU（daemon 报 no known GPU vendor）——先装 nvidia-container-toolkit"
  else
    bad "容器 GPU 测试失败：$(tail -2 /tmp/.gpu_test | tr '\n' ' ')"
  fi
fi

echo "== 3. 磁盘（Isaac Sim 镜像解压后 ~30G，加 humble/rviz 与构建缓存要 60-80G）=="
AVAIL=$(df -BG --output=avail /var/lib/docker 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -n "${AVAIL:-}" ] && [ "$AVAIL" -ge 80 ]; then ok "docker 所在分区剩余 ${AVAIL}G"
elif [ -n "${AVAIL:-}" ] && [ "$AVAIL" -ge 45 ]; then warn "剩余 ${AVAIL}G：够装最小组合，但留不出镜像重建空间，建议先清 ~/.cache"
else bad "剩余 ${AVAIL:-?}G 严重不足，必须先扩容或迁 docker data-root"; fi

echo "== 4. 显示（GUI 起来才需要；优先用 WebRTC livestream 可绕过）=="
if [ -S /tmp/.X11-unix/X${DISPLAY#*:} ] 2>/dev/null; then ok "X11 socket 存在 ($DISPLAY)"
else warn "未发现 X11 socket（DISPLAY=$DISPLAY）：若主机是 Wayland，用 livestream 而非 GUI 更稳"; fi
if command -v vulkaninfo >/dev/null 2>&1; then
  vulkaninfo --summary 2>/dev/null | grep -q "deviceName" && ok "Vulkan 可用" || bad "Vulkan 异常"
else warn "宿主机无 vulkaninfo（无所谓：容器内用的是驱动自带的 Vulkan ICD）"; fi

echo
echo "结果：$PASS 通过 / $FAIL 阻塞"
[ "$FAIL" -eq 0 ] && echo "可以进入第 4 节：拉镜像、搭容器。" || echo "先修 FAIL 项，别急着下载镜像。"
