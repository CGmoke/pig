#!/usr/bin/env bash
# 把官方 xacro 展开成 .urdf。
#
# 原则：展开命令要写死、要留痕、要可复现。
# 评审问「你是不是改了模型」时，你拿出的应该是
#   (a) 官方 xacro 的来源与版本
#   (b) 这一行确定的 xacro 参数
#   (c) scripts/check_urdf_origin.py 的通过输出
# 而不是口头解释。
#
# 官方包结构：仓库根就是 ROS 包 bajie_robot 的根（package.xml 在根目录），
# xacro 里用 $(find bajie_robot)/urdf/... 互相引用。这里构造一个最小
# ament 前缀让 find 可解析，不污染系统环境。
#
# package:// 说明：URDF 里的 mesh 路径写作 package://bajie_robot/meshes/...，
# Isaac Sim 的 URDF 导入器不解析 ROS 包 URI，所以展开后统一替换为绝对路径。
# 这只是改路径字符串，不改任何几何与 origin——合规性由 check_urdf_origin.py 背书。
#
# 用法（容器内，已 source /opt/ros/humble/setup.bash）：
#   bash scripts/expand_urdf.sh
#   XACRO_ARGS="use_sim:=true" bash scripts/expand_urdf.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${SRC:-$ROOT/src/bajie_robot/urdf/bajie_robot.urdf.xacro}"
OUT="${OUT:-$ROOT/generated/bajie_robot.urdf}"
XACRO_ARGS="${XACRO_ARGS:-}"

[ -f "$SRC" ] || { echo "找不到 xacro: $SRC（把官方资源包解到仓库根目录）" >&2; exit 1; }
command -v xacro >/dev/null || { echo "缺 xacro：apt install ros-humble-xacro" >&2; exit 1; }

# ---- 1. 构造最小 ament 前缀，让 $(find bajie_robot) 可解析 ----
# 正式工作区里 bajie_robot 是 src/ 下的真包，colcon build 后 install/setup.bash 会提供；
# 这里仍保留前缀兜底，方便在没 build 的情况下也能单独展开 xacro。
PREFIX="$ROOT/.prefix"
mkdir -p "$PREFIX/share/bajie_robot" "$PREFIX/share/ament_index/resource_index/packages"
[ -e "$PREFIX/share/bajie_robot/urdf" ]   || ln -s "$ROOT/src/bajie_robot/urdf"   "$PREFIX/share/bajie_robot/urdf"
[ -e "$PREFIX/share/bajie_robot/meshes" ] || ln -s "$ROOT/src/bajie_robot/meshes" "$PREFIX/share/bajie_robot/meshes"
[ -e "$PREFIX/share/bajie_robot/calib" ]  || ln -s "$ROOT/src/bajie_robot/calib"  "$PREFIX/share/bajie_robot/calib"
[ -f "$PREFIX/share/ament_index/resource_index/packages/bajie_robot" ] || \
  echo "bajie_robot" > "$PREFIX/share/ament_index/resource_index/packages/bajie_robot"
export AMENT_PREFIX_PATH="$PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"

# ---- 2. 展开 ----
mkdir -p "$(dirname "$OUT")"
# shellcheck disable=SC2086
xacro $XACRO_ARGS "$SRC" -o "$OUT"

# ---- 3. package:// → 绝对路径（不改几何，只改路径字符串）----
sed -i "s#package://bajie_robot/meshes#$ROOT/src/bajie_robot/meshes#g" "$OUT"

# ---- 4. 展开完立刻做结构体检 ----
python3 - "$OUT" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
links = r.findall("link"); joints = r.findall("joint")
types = {}
for j in joints:
    types[j.get("type")] = types.get(j.get("type"), 0) + 1
print(f"links={len(links)}  joints={len(joints)}  类型分布={types}")
pkg = sum(1 for m in r.iter("mesh") if (m.get("filename") or "").startswith("package://"))
print(f"残留 package:// 引用: {pkg}（应为 0）")
for j in joints:
    o = j.find("origin")
    if o is None:
        print(f"  [注意] 关节 {j.get('name')} 无 origin（默认单位变换）")
PY

echo "已生成：$OUT"
echo "下一步：在 Isaac Sim 中导入，注意不要勾 Merge Fixed Joints，"
echo "导入后跑 scripts/check_urdf_origin.py 做 origin 比对。"
