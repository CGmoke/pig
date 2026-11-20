#!/usr/bin/env python3
"""把挂进来的 OmniGraph 根 prim 改判为 ActionGraph，让它进入播放驱动的评估循环。

原理：
  Kit 里图有两层身份——`OmniGraph` 是 schema 基类（只是"这是张图"），
  `ActionGraph` 才带播放/事件驱动（OnPlaybackTick 之类真正会被 tick）。
  官方 omni_graphs/*.usd 里根 prim 只标了 OmniGraph，单独 reference 进来不会被评估，
  于是出现"节点都在、话题都在、但一帧数据都没有"。

做完三件事：
  1. 全量改判 OmniGraph -> ActionGraph
  2. 播放 + rclpy 实测 /clock、/joint_states
  3. 有数据才回写场景，避免把半成品存成正式场景

用法：
  bash -c 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh \
      /workspace/scripts/fix_graph_type.py'
"""
import time

REPORT = "/workspace/logs/fix_graph_type.txt"
SCENE = "/workspace/src/bajie_sim/scene/abr_scene_ros.usd"
ROOT = "/World/ROS2Graphs"


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd        # noqa: E402
import omni.timeline   # noqa: E402

omni.usd.get_context().open_stage(SCENE)
stage = omni.usd.get_context().get_stage()

# ---- 1. 改判类型 ----
out("=== 1. OmniGraph -> ActionGraph ===")
patched = 0
for prim in stage.Traverse():
    if str(prim.GetPath()).startswith(ROOT) and prim.GetTypeName() == "OmniGraph":
        prim.SetTypeName("ActionGraph")
        patched += 1
        out(f"   {prim.GetPath()}")
out(f"   改判 {patched} 个图根")

# ---- 2. 播放并实测 ----
out("\n=== 2. 播放并实测 20 秒 ===")
tl = omni.timeline.get_timeline_interface()
tl.play()

import rclpy                     # noqa: E402
from rclpy.node import Node      # noqa: E402
from rosgraph_msgs.msg import Clock          # noqa: E402
from sensor_msgs.msg import JointState       # noqa: E402
from tf2_msgs.msg import TFMessage           # noqa: E402

rclpy.init()
node = Node("fix_probe")
cnt = {"clock": 0, "js": 0, "tf": 0}
node.create_subscription(Clock, "/clock", lambda m: cnt.__setitem__("clock", cnt["clock"] + 1), 10)
node.create_subscription(JointState, "/joint_states", lambda m: cnt.__setitem__("js", cnt["js"] + 1), 10)
node.create_subscription(TFMessage, "/tf", lambda m: cnt.__setitem__("tf", cnt["tf"] + 1), 10)

t0 = time.time()
next_report = 5
while time.time() - t0 < 20:
    sim_app.update()
    rclpy.spin_once(node, timeout_sec=0)
    el = time.time() - t0
    if el >= next_report:
        out(f"   t={el:4.1f}s playing={tl.is_playing()} sim={tl.get_current_time():6.2f}s "
            f"clock={cnt['clock']:3d} js={cnt['js']:3d} tf={cnt['tf']:3d}")
        next_report += 5

ok = cnt["clock"] > 0 or cnt["js"] > 0
out(f"\n收包统计：clock={cnt['clock']}  joint_states={cnt['js']}  tf={cnt['tf']}")

# ---- 3. 只有通了才回写 ----
if ok:
    tl.stop()
    stage.GetRootLayer().Export(SCENE)
    out("✅ 有数据 → 已回写场景（类型改判被保留）")
else:
    out("❌ 仍无数据 → 不回写，避免污染场景。下一次要看图节点内部的输入连线是否完整")

node.destroy_node()
rclpy.shutdown()
sim_app.close()
