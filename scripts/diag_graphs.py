#!/usr/bin/env python3
"""诊断：时间轴到底有没有在跑？官方 ROS2 ActionGraph 是启用还是禁用？

背景：run_scene_ros.py 里 /clock /joint_states /tf 三个话题都已注册（说明桥接图加载了），
但收不到数据。可能有两种完全不同的原因：
  A. 时间轴没真正 play（播放状态没生效）
  B. 时间轴在跑，但 ActionGraph 节点的 state:enabled = false（图被禁用）
本脚本把两者分开测。

用法（容器内，先 source humble）：
  bash -c 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh \
      /workspace/scripts/diag_graphs.py'
"""
import sys

REPORT = "/workspace/logs/diag_graphs.txt"


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd        # noqa: E402
import omni.timeline   # noqa: E402

SCENE = "/workspace/src/bajie_sim/scene/abr_scene.usd"
out(f"打开 {SCENE}")
omni.usd.get_context().open_stage(SCENE)
stage = omni.usd.get_context().get_stage()

tl = omni.timeline.get_timeline_interface()

out("\n=== 播放前 ===")
out(f"is_playing={tl.is_playing()}  time={tl.get_current_time():.3f}")

tl.play()
out("\n=== 调用 play() 后立即 ===")
out(f"is_playing={tl.is_playing()}")

# 推进 180 帧，中途反复报告
for i in range(180):
    sim_app.update()
    if i in (0, 1, 5, 30, 90, 179):
        out(f"frame {i:3d}: is_playing={tl.is_playing()}  time={tl.get_current_time():.3f}")

# ---- ActionGraph 清单与启用状态 ----
out("\n=== ActionGraph / OmniGraph 实例 ===")
graphs = []
for prim in stage.Traverse():
    if prim.GetTypeName() == "ActionGraph":
        graphs.append(prim)

if not graphs:
    out("未找到 ActionGraph 类型 prim（可能图以 OmniGraph 子图形式存在，或在 machine/ 下未加载）")
else:
    for g in graphs:
        en = g.GetAttribute("state:enabled")
        val = en.Get() if en and en.IsValid() else "(无该属性)"
        # 统计图内节点类型
        nodes = {}
        for child in g.GetChildren():
            nodes[child.GetTypeName()] = nodes.get(child.GetTypeName(), 0) + 1
        out(f"\n{g.GetPath()}  state:enabled={val}")
        for t, c in sorted(nodes.items()):
            out(f"    {t} ×{c}")

out("\n=== 场景内所有 OgnROS2* 节点（按图统计）===")
ros_ogn = {}
for prim in stage.Traverse():
    t = prim.GetTypeName() or ""
    if "ROS2" in t:
        ros_ogn[t] = ros_ogn.get(t, 0) + 1
if ros_ogn:
    for t, c in sorted(ros_ogn.items()):
        out(f"  {t} ×{c}")
else:
    out("  (0) —— 场景内没有 ROS2 图节点，需要自己搭或引用 machine/component/omni_graphs/*.usd")

out("\n诊断结束。\n")
sim_app.close()
