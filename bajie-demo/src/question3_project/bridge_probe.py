#!/usr/bin/env python3
"""题三的 ROS 2 桥接探针，仅验证时钟和关节状态，不保存或修改官方场景。"""
import argparse
import time
from pathlib import Path

ROBOT_PATH = "/World/abr_machine/robot_root/base/base_footprint"
GRAPH_PATH = "/World/BajieROS2"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scene", required=True, type=Path, help="官方 abr_scene.usd 的路径")
args = parser.parse_args()
scene = args.scene.expanduser().resolve()
if not scene.is_file():
    parser.error(f"场景文件不存在：{scene}")


def out(msg=""):
    print(msg, flush=True)

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd                     # noqa: E402
import omni.timeline                # noqa: E402
import omni.kit.app                 # noqa: E402
import omni.graph.core as og        # noqa: E402

omni.usd.get_context().open_stage(str(scene))
stage = omni.usd.get_context().get_stage()

# 关键：建图前必须启用 bridge 扩展，否则节点类型 ID 认不出来
# （报错是 "unrecognized type ... Perhaps the extension is not loaded?"）
mgr = omni.kit.app.get_app().get_extension_manager()
if not mgr.is_extension_enabled("isaacsim.ros2.bridge"):
    mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    out("已启用 isaacsim.ros2.bridge，等待节点类型注册…")
    for _ in range(60):          # 给它几个 tick 把 OG 节点类型注册完
        sim_app.update()
else:
    out("isaacsim.ros2.bridge 已启用")

if not stage.GetPrimAtPath(ROBOT_PATH):
    out(f"!! 找不到机器人 articulation root: {ROBOT_PATH}")
    sim_app.close()
    raise SystemExit(1)
out(f"机器人 root: {ROBOT_PATH}")

out("\n=== 建图 ===")
try:
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("SubscribeJointState.outputs:jointNames",
                 "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand",
                 "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand",
                 "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand",
                 "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:targetPrim", [ROBOT_PATH]),
                ("ArticulationController.inputs:targetPrim", [ROBOT_PATH]),
                ("ArticulationController.inputs:robotPath", ROBOT_PATH),
                ("PublishJointState.inputs:topicName", "joint_states"),
            ],
        },
    )
    out("  建图成功")
except Exception as e:
    import traceback
    out(f"  建图失败: {e}")
    out(traceback.format_exc())
    sim_app.close()
    raise SystemExit(1)

# ---- 播放 + 实测 ----
out("\n=== 播放并实测 25 秒 ===")
tl = omni.timeline.get_timeline_interface()
tl.play()

import rclpy                     # noqa: E402
from rclpy.node import Node      # noqa: E402
from rosgraph_msgs.msg import Clock          # noqa: E402
from sensor_msgs.msg import JointState       # noqa: E402

rclpy.init()
node = Node("build_probe")
cnt = {"clock": 0, "js": 0}
names = []
node.create_subscription(Clock, "/clock", lambda m: cnt.__setitem__("clock", cnt["clock"] + 1), 10)


def on_js(m):
    global names
    cnt["js"] += 1
    if not names:
        names = list(m.name)


node.create_subscription(JointState, "/joint_states", on_js, 10)

t0 = time.time()
nxt = 5
while time.time() - t0 < 25:
    sim_app.update()
    rclpy.spin_once(node, timeout_sec=0)
    el = time.time() - t0
    if el >= nxt:
        out(f"   t={el:4.1f}s playing={tl.is_playing()} sim={tl.get_current_time():6.2f}s "
            f"clock={cnt['clock']:3d} js={cnt['js']:3d}")
        nxt += 5

out(f"\nclock={cnt['clock']} 条  joint_states={cnt['js']} 条")
if names:
    out(f"关节名({len(names)})：{names}")

ok = cnt["clock"] > 0 and cnt["js"] > 0
if ok:
    out("桥接状态通过：收到 /clock 和 /joint_states；尚未验证关节控制与接触。")
else:
    out("桥接状态失败：未同时收到 /clock 和 /joint_states。")

tl.stop()
node.destroy_node()
rclpy.shutdown()
sim_app.close()
raise SystemExit(0 if ok else 1)
