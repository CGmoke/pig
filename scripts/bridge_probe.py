#!/usr/bin/env python3
"""桥接探针：打开官方场景，显式启用 ROS2 bridge，实测能不能收到话题。

这是环境是否「真正可用」的最终判定：前面所有检查都只是静态检查，
只有这条能证明 /clock、/joint_states、相机话题真的在发数据。

用法（容器内）：
  bash -lc '/isaac-sim/python.sh /workspace/scripts/bridge_probe.py'

报告写入 /workspace/logs/bridge_report.txt
"""
import os
import sys

# ---- 0. 环境诊断（在 SimulationApp 之前，此时还是纯净 shell 环境）----
REPORT = "/workspace/logs/bridge_report.txt"


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()

out("=== 0. 启动前环境诊断 ===")
out(f"ROS_DISTRO = {os.environ.get('ROS_DISTRO', '(未设置)')}")
out(f"RMW_IMPLEMENTATION = {os.environ.get('RMW_IMPLEMENTATION', '(未设置)')}")
ld = os.environ.get("LD_LIBRARY_PATH", "")
out(f"LD_LIBRARY_PATH 含 humble: {'/opt/ros/humble' in ld}")
pp = os.environ.get("PYTHONPATH", "")
out(f"PYTHONPATH 含 humble: {'/opt/ros/humble' in pp}")
try:
    import rclpy  # noqa
    out("rclpy 可导入（启动前）: 是")
except Exception as e:
    out(f"rclpy 可导入（启动前）: 否 -> {e}")

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd          # noqa: E402
import omni.kit.app      # noqa: E402
import omni.timeline     # noqa: E402

# ---- 1. 显式启用 ROS2 bridge 扩展 ----
out("\n=== 1. 启用 isaacsim.ros2.bridge ===")
try:
    mgr = omni.kit.app.get_app().get_extension_manager()
    mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    out("启用: 成功")
except Exception as e:
    out(f"启用: 失败 -> {e}")

# ---- 2. 打开官方场景并播放 ----
SCENE = os.environ.get("SCENE", "/workspace/src/bajie_sim/scene/abr_scene.usd")
out(f"\n=== 2. 打开场景 {SCENE} ===")
if not omni.usd.get_context().open_stage(SCENE):
    out("!! 场景打开失败")
    sim_app.close()
    sys.exit(1)
omni.timeline.get_timeline_interface().play()
for _ in range(30):
    sim_app.update()

# ---- 3. rclpy 实测 ----
out("\n=== 3. rclpy 订阅实测 ===")
try:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("bridge_probe")

    seen = {}          # topic -> msg count
    wanted = set()

    def cb(topic):
        def _cb(msg):
            seen[topic] = seen.get(topic, 0) + 1
        return _cb

    # 先空转几帧收集话题表，再订阅感兴趣的话题
    topics_found = {}
    for i in range(240):                      # ~4 秒仿真
        sim_app.update()
        try:
            rclpy.spin_once(node, timeout_sec=0)
        except Exception:
            pass
        if i % 20 == 0:
            for name, types in node.get_topic_names_and_types():
                topics_found.setdefault(name, set()).update(types)
                if name not in wanted:
                    wanted.add(name)
                    for t in types:
                        try:
                            node.create_subscription(
                                __import__(f"{t.split('/')[0]}.msg", fromlist=["msg"]).__dict__.get(
                                    t.split("/")[1]), name, cb(name), 10)
                        except Exception:
                            pass

    out(f"发现话题 {len(topics_found)} 个:")
    for name in sorted(topics_found):
        out(f"  {name:40s} {sorted(topics_found[name])}")
    out("\n实测收到消息:")
    for topic, n in sorted(seen.items()):
        out(f"  {topic:40s} {n} 条")
    if not seen:
        out("  (0 条) —— 桥接图未在发布，检查上面节点类型注册状态")

    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    import traceback
    out(f"rclpy 实测失败: {e}")
    out(traceback.format_exc())

out("\n探针结束。\n")
sim_app.close()
