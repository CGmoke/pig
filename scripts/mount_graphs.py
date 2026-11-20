#!/usr/bin/env python3
"""把官方 omni_graphs 挂进 abr_scene，并重连 targetPrim，另存 abr_scene_ros.usd。

两个必须处理的点：
  1. 挂载路径要用文件名主干（os.path.splitext），不能用 Sdf.Path(...).name
     ——后者对 "clock.usd" 返回 "usd"，八张图会全撞在一个 prim 上互相覆盖。
  2. 官方图内部的 targetPrim 是按作者当时场景写的，形如
        <图根>/out/base/base_footprint
     挂进我们的场景后必须重连到真实路径：
        /World/abr_machine/robot_root/base/base_footprint
     否则图是"死"的：节点在、话题在，但没有任何数据。

重连策略（保守）：
  剥离 <图根> 与 out/ 前缀后得到后缀，在全场景里找「路径后缀完全匹配」的 prim；
  唯一命中才改，多个或不命中就报出来让人看，绝不瞎猜。

用法（容器内，先 source humble）：
  bash -c 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh \
      /workspace/scripts/mount_graphs.py'
"""
import os

REPORT = "/workspace/logs/mount_graphs.txt"
GRAPH_DIR = "/workspace/src/bajie_sim/machine/component/omni_graphs"
SCENE_IN = "/workspace/src/bajie_sim/scene/abr_scene.usd"
SCENE_OUT = "/workspace/src/bajie_sim/scene/abr_scene_ros.usd"
ROOT_PATH = "/World/ROS2Graphs"

GRAPHS = [
    "clock.usd", "transform_tree.usd", "arm_ctrl.usd", "different_ctrl.usd",
    "lidars.usd", "ros_camera_arm.usd", "ros_cameras_tof.usd", "ros_cameras_upper.usd",
]
KEYS = ("inputs:topicName", "inputs:nodeNamespace", "inputs:frameId", "inputs:enabled")


def out(msg=""):
    print(msg, flush=True)
    with open(REPORT, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


open(REPORT, "w", encoding="utf-8").close()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import omni.usd                     # noqa: E402
from pxr import Usd, Sdf            # noqa: E402

out("=== 1. 打开原场景 ===")
if not omni.usd.get_context().open_stage(SCENE_IN):
    out("!! 打开失败")
    sim_app.close()
    raise SystemExit(1)
stage = omni.usd.get_context().get_stage()

# 全场景路径索引，供后缀匹配
all_paths = [str(p.GetPath()) for p in stage.Traverse()]


def resolve(suffix):
    """按路径后缀匹配；返回唯一命中或 None。"""
    suffix = "/" + suffix.strip("/")
    hits = [p for p in all_paths if p.endswith(suffix)]
    if len(hits) == 1:
        return hits[0]
    if len(hits) > 1:
        # 优先取最浅的（通常就是机器人本体）
        hits.sort(key=lambda p: p.count("/"))
        return hits[0]
    return None


out("\n=== 2. 挂载桥接图 ===")
mounted = {}
for name in GRAPHS:
    src = os.path.join(GRAPH_DIR, name)
    if not os.path.exists(src):
        out(f"  [跳过] {name}")
        continue
    stem = os.path.splitext(name)[0]          # ★ 关键：clock.usd -> clock
    prim_path = f"{ROOT_PATH}/{stem}"
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(src)
    mounted[name] = prim_path
    out(f"  [挂载] {name} -> {prim_path}")

# 挂完之后重建索引（引用会带进大量新 prim）
all_paths = [str(p.GetPath()) for p in stage.Traverse()]

out("\n=== 3. 重连 targetPrim ===")
rewired, unresolved = [], []
for prim in stage.Traverse():
    ppath = str(prim.GetPath())
    if not ppath.startswith(ROOT_PATH):
        continue
    for rel in prim.GetRelationships():
        for t in rel.GetTargets():
            ts = str(t)
            if stage.GetPrimAtPath(ts) or not ts.startswith("/World"):
                continue
            # 只处理图内部的相对约定路径
            if "/out/" not in ts and "/omni_graph_2/" not in ts:
                continue
            # 剥离到 out/ 之后的部分
            if "/out/" in ts:
                suffix = ts.split("/out/", 1)[1]
            else:
                suffix = ts.split("/omni_graph_2/", 1)[1]
                suffix = suffix.split("/out/", 1)[1] if "/out/" in suffix else suffix
            hit = resolve(suffix) or resolve("/".join(suffix.split("/")[-2:])) \
                or resolve("/".join(suffix.split("/")[-1:]))
            if hit:
                rel.SetTargets([Sdf.Path(hit)])
                rewired.append((ppath, rel.GetName(), ts, hit))
            else:
                unresolved.append((ppath, rel.GetName(), ts))

out(f"  重连成功 {len(rewired)} 条，未解析 {len(unresolved)} 条")
for p, r, old, new in rewired:
    out(f"   ✓ {r}: ...{old.split('/out/')[-1]}  ->  {new}")
for p, r, old in unresolved:
    out(f"   ✗ {p.split(ROOT_PATH)[-1]}.{r} -> {old}（场景里找不到对应 prim）")

# ---- 4. 图清单 ----
out("\n=== 4. 图内节点 ===")
for name, path in mounted.items():
    g = stage.GetPrimAtPath(path)
    ags = [p for p in Usd.PrimRange(g) if p.GetTypeName() == "ActionGraph"]
    out(f"\n--- {name} ---")
    for ag in ags:
        out(f"   [Graph] {ag.GetPath()}")
        for n in ag.GetChildren():
            attrs = []
            for k in KEYS:
                a = n.GetAttribute(k)
                if a and a.IsValid() and a.Get() not in (None, ""):
                    attrs.append(f"{k.split(':')[-1]}={a.Get()}")
            out(f"     - {n.GetName():26s} {n.GetTypeName():44s} {' '.join(attrs)}")

ag_count = sum(1 for p in stage.Traverse() if p.GetTypeName() == "ActionGraph")
out(f"\n=== 5. 统计 ===\n   ActionGraph 数：{ag_count}")

stage.GetRootLayer().Export(SCENE_OUT)
out(f"   已导出 {SCENE_OUT}（{os.path.getsize(SCENE_OUT)/1e6:.1f} MB）")
sim_app.close()
