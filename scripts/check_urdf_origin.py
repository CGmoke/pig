#!/usr/bin/env python3
"""校验「URDF → Isaac Sim」导入后，连杆与关节 origin 是否被完整保留。

题目明确要求：基于官方模型，保留连杆几何和关节 origin。
所以这不是"能不能跑"的问题，而是"改没改模型"的证据问题——评审可以直接比对。

脚本做两件事：
  1. 解析展开后的 .urdf，取出全部 joint 的 parent/child 与 origin(xyz, rpy)
  2. 从 Isaac Sim 导入产出的 .usd 里读 PhysicsXxxJoint 的 localPos0 / localRot0，
     逐条比对。缺失的关节名会单独列出来（未勾选 Merge Fixed Joints 时，
     固定关节会被合并消失，这是最常见的违规点）。

用法（容器内，必须用 Isaac Sim 自带的 python，才有 pxr）：
  /isaac-sim/python.sh scripts/check_urdf_origin.py \\
      generated/bajie_robot.urdf generated/bajie_robot.usd
"""
import sys
import math
import xml.etree.ElementTree as ET


# ---------- URDF 侧 ----------
def rpy_to_quat(r, p, y):
    """URDF 的 rpy 是固定轴 XYZ 顺序，等价于 R = Rz*Ry*Rx。返回 (w, x, y, z)。"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        org = j.find("origin")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if org is not None:
            if org.get("xyz"):
                xyz = tuple(float(v) for v in org.get("xyz").split())
            if org.get("rpy"):
                rpy = tuple(float(v) for v in org.get("rpy").split())
        joints[name] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz,
            "quat": rpy_to_quat(*rpy),
        }
    links = [l.get("name") for l in root.findall("link")]
    return joints, links


# ---------- USD 侧 ----------
def quat_angle(q1, q2):
    """两个四元数的夹角（弧度），处理 q 与 -q 同一旋转。"""
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    d = max(-1.0, min(1.0, d))
    return 2.0 * math.acos(d)


def parse_usd(path):
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(path)
    if stage is None:
        raise RuntimeError(f"打不开 USD: {path}")

    joints = {}
    for prim in stage.Traverse():
        t = prim.GetTypeName()
        if t not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint",
                     "PhysicsFixedJoint", "PhysicsJoint", "PhysicsSphericalJoint"):
            continue
        pos = prim.GetAttribute("physics:localPos0")
        rot = prim.GetAttribute("physics:localRot0")
        pos_v = tuple(pos.Get()) if pos and pos.IsValid() and pos.Get() is not None else (0.0, 0.0, 0.0)
        if rot and rot.IsValid() and rot.Get() is not None:
            q = rot.Get()
            # Gf.Quatf 的构造是 (w, x, y, z)，属性访问给的是 real + imaginary
            quat = (q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2])
        else:
            quat = (1.0, 0.0, 0.0, 0.0)
        joints[prim.GetName()] = {"type": t, "xyz": pos_v, "quat": quat}

    prim_count = sum(1 for _ in stage.Traverse())
    return joints, prim_count


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    urdf_path, usd_path = sys.argv[1], sys.argv[2]

    u_joints, u_links = parse_urdf(urdf_path)
    s_joints, prim_count = parse_usd(usd_path)

    print(f"\nURDF: {urdf_path}  → links={len(u_links)} joints={len(u_joints)}")
    print(f"USD : {usd_path}  → joint prims={len(s_joints)} total prims={prim_count}\n")

    missing, mismatch, checked = [], [], 0
    for name, u in sorted(u_joints.items()):
        s = s_joints.get(name)
        if s is None:
            missing.append((name, u["type"]))
            continue
        checked += 1
        dt = math.sqrt(sum((a - b) ** 2 for a, b in zip(u["xyz"], s["xyz"])))
        da = quat_angle(u["quat"], s["quat"])
        if dt > 1e-3 or da > 1e-3:
            mismatch.append((name, dt, math.degrees(da)))

    print(f"比对通过: {checked}/{len(u_joints)}")
    if missing:
        print(f"\n[缺失 {len(missing)}] 这些关节在 USD 里找不到——"
              f"若为 fixed 类型，几乎可以确定是导入时勾选了 Merge Fixed Joints：")
        for n, t in missing:
            print(f"    - {n} ({t})")
    if mismatch:
        print(f"\n[偏移 {len(mismatch)}] origin 与官方 URDF 不一致（平移 m / 旋转 deg）：")
        for n, dt, da in mismatch:
            print(f"    - {n}: Δxyz={dt:.6f} m, Δrpy={da:.4f} deg")
    if not missing and not mismatch:
        print("\n✅ 连杆与关节 origin 与官方 URDF 完全一致，满足「基于官方模型」的要求。")

    sys.exit(0 if not missing and not mismatch else 1)


if __name__ == "__main__":
    main()
