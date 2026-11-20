#!/usr/bin/env python3
"""把官方 omni_graphs/*.usd 的内部结构导出来：图里有哪些节点、各自的关键参数。

不需要启动 Isaac Sim，只用 pxr 读 USD，秒级返回。

用法（容器内）：
  /isaac-sim/python.sh scripts/dump_graphs.py
  /isaac-sim/python.sh scripts/dump_graphs.py /workspace/src/bajie_sim/machine/component/omni_graphs/clock.usd
"""
import glob
import os
import sys

from pxr import Usd, Sdf

DEFAULT_DIR = "/workspace/src/bajie_sim/machine/component/omni_graphs"
# 优先看这几个属性：它们决定话题名、频率、是否启用
KEYS = ("topicName", "nodeNamespace", "frameId", "enabled", "type", "topicType", "nodeType")


def dump(path):
    stage = Usd.Stage.Open(path)
    if stage is None:
        print(f"!! 打不开 {path}")
        return

    print(f"\n{'='*70}\n{os.path.basename(path)}")
    default = stage.GetDefaultPrim()
    print(f"默认 Prim: {default.GetPath() if default else '(无)'}")

    graphs, nodes = [], []
    for prim in stage.Traverse():
        t = prim.GetTypeName() or ""
        if t == "ActionGraph":
            graphs.append(prim)
        elif prim.GetParent() and (prim.GetParent().GetTypeName() == "ActionGraph"):
            nodes.append(prim)

    if not graphs:
        print("  (无 ActionGraph)")
        return

    for g in graphs:
        print(f"\n  [Graph] {g.GetPath()}")
        kids = [c for c in g.GetChildren()]
        print(f"     节点数 {len(kids)}")
        for n in kids:
            t = n.GetTypeName()
            attrs = []
            for name in KEYS:
                a = n.GetAttribute(name)
                if a and a.IsValid():
                    v = a.Get()
                    if v not in (None, ""):
                        attrs.append(f"{name}={v}")
            # 图内连线
            inputs = [p.GetName() for p in n.GetChildren()
                      if p.GetName().endswith("inputs")]
            print(f"       - {n.GetName():28s} {t}")
            if attrs:
                print(f"           {'  '.join(attrs)}")
            if inputs:
                print(f"           输入端口: {inputs}")

    # 连线和 target 路径（谁驱动谁）
    print("\n  [连接关系]")
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            tgt = rel.GetTargets()
            if tgt:
                print(f"    {prim.GetPath()}.{rel.GetName()} -> {[str(t) for t in tgt]}")


def main():
    if len(sys.argv) > 1:
        paths = sys.argv[1:]
    else:
        paths = sorted(glob.glob(os.path.join(DEFAULT_DIR, "*.usd")))
    print(f"共 {len(paths)} 个图文件")
    for p in paths:
        dump(p)


if __name__ == "__main__":
    main()
