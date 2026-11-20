#!/usr/bin/env python3
"""诊断挂载后的图：prim 树、state:enabled、激活后能否发数据。

背景：mount_graphs.py 已把 8 张图挂进 /World/ROS2Graphs/* 并重连了 12 条 targetPrim，
但 run_scene_ros.py 仍收不到 /clock。可能原因：
  A. 图根没有 ActionGraph 类型 → OmniGraph 运行时不会评估它
  B. 图根 state:enabled = false
本脚本先"看"，再"改"（尝试置 enabled=True），最后实测。

用法：
  bash -c 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh \
      /workspace/scripts/diag_mounted.py'
"""
import time

REPORT = "/workspace/logs/diag_mounted.txt"


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd        # noqa: E402
import omni.timeline   # noqa: E402

SCENE = "/workspace/src/bajie_sim/scene/abr_scene_ros.usd"
omni.usd.get_context().open_stage(SCENE)
stage = omni.usd.get_context().get_stage()

ROOT = "/World/ROS2Graphs"
out("=== 1. 挂载点下的 prim 树（前 3 层）===")
root_prim = stage.GetPrimAtPath(ROOT)
if not root_prim:
    out("!! /World/ROS2Graphs 不存在")
for child in root_prim.GetChildren():
    out(f"\n{child.GetPath()}  type={child.GetTypeName()}")
    for gc in child.GetChildren():
        en = gc.GetAttribute("state:enabled")
        out(f"   {gc.GetName():28s} type={gc.GetTypeName():16s} "
            f"enabled={en.Get() if en and en.IsValid() else '-'}")
        for ggc in list(gc.GetChildren())[:6]:
            out(f"      . {ggc.GetName():24s} type={ggc.GetTypeName()}")

# ---- 2. 尝试把所有图根的 state:enabled 置 True ----
out("\n=== 2. 强制启用 ===")
changed = 0
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith(ROOT):
        continue
    a = prim.GetAttribute("state:enabled")
    if a and a.IsValid():
        if not a.Get():
            a.Set(True)
            changed += 1
            out(f"   enabled=True: {prim.GetPath()}")
out(f"   共启用 {changed} 个节点")

# ---- 3. 播放 + 实测 ----
out("\n=== 3. 播放并实测 ===")
tl = omni.timeline.get_timeline_interface()
tl.play()

import rclpy                     # noqa: E402
from rclpy.node import Node      # noqa: E402
from rosgraph_msgs.msg import Clock          # noqa: E402
from sensor_msgs.msg import JointState       # noqa: E402

rclpy.init()
node = Node("diag_mounted")
counts = {"clock": 0, "js": 0}
node.create_subscription(Clock, "/clock", lambda m: counts.__setitem__("clock", counts["clock"] + 1), 10)
node.create_subscription(JointState, "/joint_states",
                         lambda m: counts.__setitem__("js", counts["js"] + 1), 10)

t0 = time.time()
while time.time() - t0 < 20:
    sim_app.update()
    rclpy.spin_once(node, timeout_sec=0)
    if int(time.time() - t0) in (5, 10, 15, 19):
        out(f"   t={int(time.time()-t0)}s  playing={tl.is_playing()}  sim_time={tl.get_current_time():.2f}"
            f"  clock={counts['clock']}  joint_states={counts['js']}")

out(f"\n结束：clock={counts['clock']} 条，joint_states={counts['js']} 条")
node.destroy_node()
rclpy.shutdown()
sim_app.close()
