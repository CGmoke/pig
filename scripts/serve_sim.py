#!/usr/bin/env python3
"""仿真服务：官方场景 + 运行时注入 ROS 2 桥接图，常驻供外部节点调用。

设计取舍（重要）：
  不把图写进 .usd，而是每次启动时用 og.Controller 现搭。
  理由：
    1. og.Controller 默认编辑落在 session layer，Export 到磁盘会丢节点（实测过）；
    2. 保持 abr_scene.usd 原封不动，符合"基于官方场景"的合规要求，
       桥接属于运行时行为，不修改官方资产。

注入的图（T1 最小闭环）：
  OnPlaybackTick ─┬→ ROS2PublishClock        → /clock
                  ├→ ROS2PublishJointState   → /joint_states
                  └→ ArticulationController  ← /joint_command（经 ROS2SubscribeJointState）

用法（容器内，后台）：
  bash -c 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh \
      /workspace/scripts/serve_sim.py'
"""
import signal
import sys

SCENE = "/workspace/src/bajie_sim/scene/abr_scene.usd"
ROBOT = "/World/abr_machine/robot_root/base/base_footprint"
GRAPH = "/World/BajieROS2"

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd                  # noqa: E402
import omni.timeline             # noqa: E402
import omni.kit.app              # noqa: E402
import omni.graph.core as og     # noqa: E402

# 1) 先打开官方场景（顺序很讲究：build_bridge 里"先开场景再启用扩展"才成功）
omni.usd.get_context().open_stage(SCENE)
stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath(ROBOT):
    print(f"[serve_sim] !! 找不到 {ROBOT}", flush=True)
    sys.exit(1)
print(f"[serve_sim] 场景已加载：{SCENE}", flush=True)

# 2) 再启用 bridge 扩展，并给它几个 tick 注册节点类型
mgr = omni.kit.app.get_app().get_extension_manager()
if not mgr.is_extension_enabled("isaacsim.ros2.bridge"):
    mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    print("[serve_sim] 已启用 isaacsim.ros2.bridge", flush=True)
    for _ in range(60):
        sim_app.update()

# 3) 注入桥接图
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
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
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:targetPrim", [ROBOT]),
            ("ArticulationController.inputs:targetPrim", [ROBOT]),
            ("ArticulationController.inputs:robotPath", ROBOT),
            ("PublishJointState.inputs:topicName", "joint_states"),
        ],
    },
)
print(f"[serve_sim] 桥接图已注入：{GRAPH}", flush=True)

# 4) 播放
tl = omni.timeline.get_timeline_interface()
tl.play()
print("[serve_sim] 已开始播放，等待外部指令…", flush=True)

stop = {"flag": False}


def handler(signum, frame):
    stop["flag"] = True
    print("[serve_sim] 收到退出信号", flush=True)


signal.signal(signal.SIGTERM, handler)
signal.signal(signal.SIGINT, handler)

n = 0
while not stop["flag"]:
    sim_app.update()
    n += 1
    if n % 300 == 0:
        print(f"[serve_sim] frame={n} sim_time={tl.get_current_time():.2f}s", flush=True)

sim_app.close()
sys.exit(0)
