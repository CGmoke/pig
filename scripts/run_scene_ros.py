#!/usr/bin/env python3
"""无头运行器：开官方场景 → 播放时间轴 → 用 rclpy 实测话题数据。

为什么需要它：
  1. 本机这个 experience（isaacsim.exp.full.streaming.kit）只提供 WSS 信令端点(49100)，
     不提供浏览器网页客户端，因此「打开网页点 Play」这条路走不通；
  2. 官方 ROS2 图全部挂在 OnPlaybackTick 上，不播不发——必须有人按播放；
  3. 比赛开发本来就需要一个可脚本化、可复现的跑法，而不是每次手点 GUI。

用法（容器内，必须先 source humble）：
  bash -c 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh \
      /workspace/scripts/run_scene_ros.py --duration 10'

参数：
  --scene     场景路径，默认 /workspace/src/bajie_sim/scene/abr_scene.usd
  --duration  播放的仿真秒数，默认 10
  --report    报告路径，默认 /workspace/logs/ros_live_report.txt
"""
import argparse
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="/workspace/src/bajie_sim/scene/abr_scene.usd")
parser.add_argument("--duration", type=float, default=10.0)
parser.add_argument("--report", default="/workspace/logs/ros_live_report.txt")
args, _ = parser.parse_known_args()

REPORT = args.report


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd        # noqa: E402
import omni.timeline   # noqa: E402

out("=== 1. 打开场景 ===")
out(f"scene = {args.scene}")
if not omni.usd.get_context().open_stage(args.scene):
    out("!! 打开失败")
    sim_app.close()
    sys.exit(1)

stage = omni.usd.get_context().get_stage()
dp = stage.GetDefaultPrim()
out(f"默认 Prim = {dp.GetPath() if dp else '(无)'}")

out("\n=== 2. 播放时间轴 ===")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
out(f"is_playing = {timeline.is_playing()}")

# ---- 3. rclpy 实测 ----
out("\n=== 3. rclpy 订阅 ===")
import rclpy                     # noqa: E402
from rclpy.node import Node      # noqa: E402
from rosgraph_msgs.msg import Clock          # noqa: E402
from sensor_msgs.msg import JointState       # noqa: E402
from tf2_msgs.msg import TFMessage           # noqa: E402

rclpy.init()
node = Node("bajie_probe")

counts = {}
last = {}


def make_cb(topic):
    def cb(msg):
        counts[topic] = counts.get(topic, 0) + 1
        if topic == "/clock":
            last["sim_time"] = msg.clock.sec + msg.clock.nanosec * 1e-9
        elif topic == "/joint_states":
            last["joints"] = list(msg.name)
    return cb


subs = [
    node.create_subscription(Clock, "/clock", make_cb("/clock"), 10),
    node.create_subscription(JointState, "/joint_states", make_cb("/joint_states"), 10),
    node.create_subscription(TFMessage, "/tf", make_cb("/tf"), 10),
]

# 播放 args.duration 秒（按仿真时钟推进，墙钟上限 3 倍，避免卡死）
t0 = time.time()
sim_elapsed = 0.0
while sim_elapsed < args.duration and (time.time() - t0) < args.duration * 3:
    sim_app.update()
    try:
        rclpy.spin_once(node, timeout_sec=0)
    except Exception:
        pass
    sim_elapsed = last.get("sim_time", 0.0)

topics = {}
for name, types in node.get_topic_names_and_types():
    topics[name] = types

out(f"\n场景内可见话题 {len(topics)} 个：")
for name in sorted(topics):
    flag = " ←有数据" if counts.get(name) else ""
    out(f"  {name:44s} {topics[name]}{flag}")

out("\n实测收包：")
for topic in ("/clock", "/joint_states", "/tf"):
    out(f"  {topic:16s} {counts.get(topic, 0):5d} 条")
out(f"\n仿真时钟推进到 {sim_elapsed:.2f}s（目标 {args.duration}s）")
if last.get("joints"):
    out(f"关节名({len(last['joints'])})：{last['joints']}")

ok = counts.get("/clock", 0) > 0 and counts.get("/joint_states", 0) > 0
out("\n结论：" + ("✅ 桥接链路正常，/clock 与 /joint_states 都在发数据"
                 if ok else "❌ 未收到数据，检查是否 source humble 后启动"))

node.destroy_node()
rclpy.shutdown()
timeline.stop()
sim_app.close()
sys.exit(0 if ok else 1)
