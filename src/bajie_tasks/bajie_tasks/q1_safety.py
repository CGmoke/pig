"""题一 RViz 轨迹预检：用官方 URDF 的局部包围盒保守验证整段运动。"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = (
    'lift_joint', 'pitch_joint', *(f'joint{i}' for i in range(1, 7)),
    'arm_finger_joint1', 'arm_finger_joint2',
)
ARM_LINKS = (
    *(f'arm{i}_link' for i in range(1, 7)), 'gripper_link',
    'gripper_left_link', 'gripper_right_link', 'berxel_arm_base_link',
)
BASE = 'base_footprint'
GROUND_CLEARANCE = 0.05  # 地面是 base_footprint 下的 z=0 平面
BODY_CLEARANCE = 0.02
ZERO = dict.fromkeys(JOINTS, 0.0)


def _vector(value):
    return np.array(value.split(), dtype=float)


def _rotation(axis, angle):
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def _origin(element):
    origin = element.find('origin')
    if origin is None:
        return np.eye(3), np.zeros(3)
    roll, pitch, yaw = _vector(origin.get('rpy', '0 0 0'))
    return (_rotation((0, 0, 1), yaw) @ _rotation((0, 1, 0), pitch)
            @ _rotation((1, 0, 0), roll), _vector(origin.get('xyz', '0 0 0')))


def _stl_vertices(path):
    # 官方 URDF 引用二进制 STL；只读取，不修改模型。
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    if not count or len(data) != 84 + 50 * count:
        raise ValueError(f'不是官方模型所用的二进制 STL：{path}')
    layout = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(data, dtype=layout, offset=84)['vertices'].reshape(-1, 3)


def _local_bounds(element, robot_share):
    geometry = element.find('geometry')[0]
    r, t = _origin(element)
    if geometry.tag == 'mesh':
        uri = geometry.attrib['filename']
        prefix = 'package://bajie_robot/'
        path = (robot_share / uri[len(prefix):]) if uri.startswith(prefix) else Path(uri)
        v = _stl_vertices(path) * _vector(geometry.get('scale', '1 1 1'))
        v = v @ r.T + t
        return v.min(axis=0), v.max(axis=0)
    if geometry.tag == 'box':
        half = _vector(geometry.attrib['size']) / 2
    elif geometry.tag == 'cylinder':
        radius = float(geometry.attrib['radius'])
        half = np.array((radius, radius, float(geometry.attrib['length']) / 2))
    elif geometry.tag == 'sphere':
        half = np.full(3, float(geometry.attrib['radius']))
    else:
        raise ValueError(f'无法预检未知几何：{geometry.tag}')
    projected = abs(r) @ half
    return t - projected, t + projected


class UnsafeMotion(ValueError):
    """轨迹不能在要求的净空下通过预检。"""


class ClearanceModel:
    def __init__(self, robot_description, robot_share):
        root = ET.fromstring(robot_description)
        children = {}
        self.limits = {}
        for joint in root.findall('joint'):
            name, kind = joint.attrib['name'], joint.attrib['type']
            if kind != 'fixed':
                if name not in JOINTS or kind not in ('revolute', 'prismatic'):
                    raise ValueError(f'无法预检未知活动关节：{name} ({kind})')
                limit = joint.find('limit')
                self.limits[name] = (float(limit.attrib['lower']), float(limit.attrib['upper']))
            axis = joint.find('axis')
            axis = _vector(axis.get('xyz', '1 0 0') if axis is not None else '1 0 0')
            if kind != 'fixed':
                axis = axis / np.linalg.norm(axis)
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            r, t = _origin(joint)
            index = JOINTS.index(name) if kind != 'fixed' else None
            children.setdefault(parent, []).append((name, index, child, kind, axis, r, t))
        if set(self.limits) != set(JOINTS):
            raise ValueError('官方模型缺少题一关节')
        self.joints = []
        paths = {BASE: []}
        order = [BASE]
        for parent in order:
            for item in children.get(parent, ()):
                child = item[2]
                if child in paths:
                    raise ValueError(f'无效 URDF 关节树：{child} 重复出现')
                self.joints.append((parent, *item))
                paths[child] = [*paths[parent], item]
                order.append(child)

        self.names = []
        centres, halves, reach = [], [], []
        for link in root.findall('link'):
            name = link.attrib['name']
            bounds = [_local_bounds(g, Path(robot_share)) for g in
                      link.findall('visual') + link.findall('collision')]
            if not bounds:
                continue
            lo = np.min([b[0] for b in bounds], axis=0)
            hi = np.max([b[1] for b in bounds], axis=0)
            self.names.append(name)
            centres.append((hi + lo) / 2)
            halves.append((hi - lo) / 2)

            # 点到祖先旋转轴的保守距离：本包围盒最远角 + 下游关节的平移长度。
            # 旋转 Δq 时，任一点移动不超过 radius * |Δq|；升降则直接是 |Δq|。
            radius = np.linalg.norm(np.maximum(abs(lo), abs(hi)))
            coeff = np.zeros(len(JOINTS))
            for joint, index, _, kind, _, _, t in reversed(paths[name]):
                if kind == 'revolute':
                    coeff[index] = radius
                elif kind == 'prismatic':
                    coeff[index] = 1.0
                    radius += max(map(abs, self.limits[joint]))
                radius += np.linalg.norm(t)
            reach.append(coeff)
        self.centres, self.halves, self.reach = map(np.array, (centres, halves, reach))
        self.arm = np.array([self.names.index(link) for link in ARM_LINKS])
        # arm1 与固定安装座本来就是相连的关节接口；其它臂段仍检查与安装座的间距。
        pairs = [(i, j) for i in self.arm for j, name in enumerate(self.names)
                 if name not in ARM_LINKS and (self.names[i], name) != ('arm1_link', 'arm_base_link')]
        self.arm_pair, self.body_pair = np.array(pairs).T

    def gaps(self, q):
        """每个臂段对地面、以及每个臂段/机身包围盒对的间隔下界。"""
        frames = {BASE: (np.eye(3), np.zeros(3))}
        for parent, _, i, child, kind, axis, r, t in self.joints:
            rp, tp = frames[parent]
            if kind == 'revolute':
                r = r @ _rotation(axis, q[i])
            elif kind == 'prismatic':
                t = t + r @ axis * q[i]
            frames[child] = rp @ r, tp + rp @ t
        r, t = map(np.array, zip(*(frames[name] for name in self.names)))
        c = (r @ self.centres[..., None])[..., 0] + t
        e = (abs(r) @ self.halves[..., None])[..., 0]
        a, b = self.arm_pair, self.body_pair
        return c[self.arm, 2] - e[self.arm, 2], np.max(abs(c[a] - c[b]) - e[a] - e[b], axis=1)

    def certify(self, start, end):
        """覆盖整条关节插值段（不只检查离散发布帧）；返回地面/机身净空下界。"""
        def vector(state):
            q = np.array([state[name] for name in JOINTS])
            for i, name in enumerate(JOINTS):
                lo, hi = self.limits[name]
                if not math.isfinite(q[i]) or not lo <= q[i] <= hi:
                    raise UnsafeMotion(f'{name} 超出 URDF 限位：{q[i]}')
            return q

        def interval(a, b, depth=0):
            ground, body = self.gaps((a + b) / 2)
            ig, ib = int(np.argmin(ground)), int(np.argmin(body))
            if ground[ig] < GROUND_CLEARANCE:
                raise UnsafeMotion(f'{self.names[self.arm[ig]]} 对地面净空不足 ({ground[ig]:.3f} m)')
            if body[ib] < BODY_CLEARANCE:
                l, r = self.arm_pair[ib], self.body_pair[ib]
                raise UnsafeMotion(f'{self.names[l]} 对 {self.names[r]} 净空不足 ({body[ib]:.3f} m)')
            travel = self.reach @ (abs(b - a) / 2)
            lower = np.array(((ground - travel[self.arm]).min(),
                              (body - travel[self.arm_pair] - travel[self.body_pair]).min()))
            if np.all(lower >= (GROUND_CLEARANCE, BODY_CLEARANCE)):
                return lower
            if depth >= 12:
                raise UnsafeMotion('无法证明整段运动满足净空要求')
            midpoint = (a + b) / 2
            return np.minimum(interval(a, midpoint, depth + 1), interval(midpoint, b, depth + 1))

        return interval(vector(start), vector(end))
