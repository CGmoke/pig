#!/usr/bin/env python3
"""场景体检：打开 USD，把「机械臂结构 + 已有 ROS 2 桥接」摸清楚。

为什么第一步做这个而不是直接写控制代码：
  abr_scene.usd 里已经内置了 ActionGraph（PublishJointState / Clock / TF / 相机），
  你再搭一遍就会双发 TF、双发 /clock，症状是 TF 树抖动、时间戳跳变——排查成本极高。
  先看清已有节点，再决定"补"还是"建"。

用法（容器内，必须先 source /opt/ros/humble/setup.bash，否则 ROS2 节点类型注册不上）：
  bash -lc '/isaac-sim/python.sh scripts/smoke_sim.py'
  bash -lc '/isaac-sim/python.sh scripts/smoke_sim.py /workspace/src/bajie_sim/scene/abr_scene.usd'

注意：Isaac Sim 会把 Python stdout 重定向进自己的 kit 日志，所以报告同时
写入 /workspace/logs/smoke_report.txt，以文件为准。
"""
import sys
import os

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import omni.usd                     # noqa: E402
from pxr import Usd, UsdPhysics     # noqa: E402

usd_path = sys.argv[1] if len(sys.argv) > 1 else os.environ.get(
    "SCENE", "/workspace/src/bajie_sim/scene/abr_scene.usd")

REPORT = "/workspace/logs/smoke_report.txt"


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()   # 每次重写

out(f"=== 打开: {usd_path} ===")
ok = omni.usd.get_context().open_stage(usd_path)
if not ok:
    out("!! 打开失败：检查路径、以及 USD 引用的外部资源是否随资源包一起到位")
    sim_app.close()
    sys.exit(1)

stage = omni.usd.get_context().get_stage()
dp = stage.GetDefaultPrim()
out(f"默认 Prim: {dp.GetPath() if dp else '(无)'}")

# --- 1. 机器人关节结构 ---
joints, arts = [], []
for prim in stage.Traverse():
    t = prim.GetTypeName()
    if t in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint", "PhysicsJoint"):
        joints.append(prim)
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        arts.append(prim)

out(f"\n=== Articulation Root: {len(arts)} ===")
for a in arts:
    out(f"  {a.GetPath()}  ({a.GetTypeName()})")

out(f"\n=== 关节: {len(joints)} ===")
for j in joints:
    pos = j.GetAttribute("physics:localPos0")
    p = tuple(round(v, 5) for v in pos.Get()) if pos and pos.IsValid() and pos.Get() else None
    out(f"  {j.GetName():32s} {j.GetTypeName():22s} localPos0={p}")

# --- 2. 已有 ROS 2 节点（决定要不要自己搭桥） ---
out("\n=== 场景内已有 ROS2 节点 ===")
ros_nodes = []
for prim in stage.Traverse():
    t = prim.GetTypeName() or ""
    if "ROS2" in t:
        ros_nodes.append(prim)
if not ros_nodes:
    out("  (无) —— 需要自己搭 ActionGraph，见 docs/开发环境搭建.md 第 6 节")
else:
    names = {}
    for n in ros_nodes:
        names[n.GetTypeName()] = names.get(n.GetTypeName(), 0) + 1
    for k, v in sorted(names.items()):
        out(f"  {k} ×{v}")

# --- 3. 相机 / 传感器（T2、T3 的输入源） ---
out("\n=== 相机 / 传感器 ===")
for prim in stage.Traverse():
    t = prim.GetTypeName()
    if t and ("Camera" in t or "Lidar" in t or "Imu" in t):
        out(f"  {prim.GetPath()}  ({t})")

out("\n体检完成。把本报告连同 check_urdf_origin.py 的输出一起放进提交材料的技术说明，"
    "就是「基于官方场景」的证据。\n")
sim_app.close()
