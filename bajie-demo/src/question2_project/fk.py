"""URDF 串联链的刚体变换。T_A_B 将 B 系下的坐标映射到 A 系。"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_rotation(roll, pitch, yaw):
    """ROS 固定轴 RPY：Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _transform(xyz, rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = xyz
    return matrix


def finite_number(value, label):
    if type(value) not in (int, float):
        raise ValueError(f'{label} 必须是有限数值')
    try:
        if math.isfinite(value):
            return float(value)
    except OverflowError:
        pass
    raise ValueError(f'{label} 必须是有限数值')


def _pose_vector(pose, key):
    values = pose.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f'{key} 必须是由三个有限数值组成的数组')
    return [finite_number(value, f'{key}[{index}]') for index, value in enumerate(values)]


def pose_to_matrix(pose):
    """将含 xyz（米）和 rpy（弧度）的字典转为 4×4 矩阵。"""
    if not isinstance(pose, dict):
        raise ValueError('位姿必须是含 xyz 和 rpy 的对象')
    return _transform(_pose_vector(pose, 'xyz'), _rpy_rotation(*_pose_vector(pose, 'rpy')))


def _rotation_to_quat(rotation):
    """直接从矩阵取四元数，选择最大的分量作为除数。"""
    r = rotation
    trace = np.trace(r)
    if trace > 0:
        s = 2 * math.sqrt(1 + trace)
        quat = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
        quat = [0.0] * 4
        quat[i] = s / 4
        quat[j] = (r[j, i] + r[i, j]) / s
        quat[k] = (r[k, i] + r[i, k]) / s
        quat[3] = (r[k, j] - r[j, k]) / s
    normalized = np.asarray(quat) / math.hypot(*quat)
    return -normalized if normalized[3] < 0 else normalized


def matrix_to_pose(matrix):
    rotation = matrix[:3, :3]
    cos_p = math.hypot(rotation[0, 0], rotation[1, 0])
    sin_p = -rotation[2, 0]
    pitch = math.atan2(sin_p, cos_p)
    if cos_p > 1e-13:
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        # 将 Y 轴先逆偏航，再逆俯仰。这样求 roll 不必从两个
        # 接近零（均含 cos(pitch)）的矩阵元素分别估计角度。
        sy, cy = math.sin(yaw), math.cos(yaw)
        yx, yy, yz = rotation[:, 1]
        roll = math.atan2(math.sin(pitch) * (cy * yx + sy * yy) + math.cos(pitch) * yz,
                          -sy * yx + cy * yy)
    else:
        # 俯仰接近 ±π/2 时，固定 roll=0，避免从近零元素误算 yaw。
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return {'xyz': matrix[:3, 3].tolist(), 'rpy': [roll, pitch, yaw],
            'quat_xyzw': _rotation_to_quat(rotation).tolist()}


def rigid_inverse(matrix):
    rotation_t = matrix[:3, :3].T
    return _transform(-rotation_t @ matrix[:3, 3], rotation_t)


def _urdf_vector(element, attribute, default):
    text = default if element is None else element.get(attribute, default)
    values = [float(part) for part in text.split()]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f'URDF 属性 {attribute} 需要三个有限数值：{text}')
    return np.array(values)


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray

    @classmethod
    def from_xml(cls, joint):
        name, kind = joint.attrib['name'], joint.attrib['type']
        if kind not in ('fixed', 'prismatic', 'revolute', 'continuous') or joint.find('mimic') is not None:
            raise ValueError(f'链上关节 {name} 的类型或 mimic 配置不受支持')
        origin = joint.find('origin')
        xyz = _urdf_vector(origin, 'xyz', '0 0 0')
        rpy = _urdf_vector(origin, 'rpy', '0 0 0')
        axis = _urdf_vector(joint.find('axis'), 'xyz', '1 0 0')
        if kind != 'fixed':
            norm = math.hypot(*axis)
            if not math.isfinite(norm) or norm == 0:
                raise ValueError(f'链上关节 {name} 的轴无效')
            axis /= norm
        return cls(name, kind, _transform(xyz, _rpy_rotation(*rpy)), axis)

    def at(self, positions):
        if self.kind == 'fixed':
            return self.origin
        if self.name not in positions:
            raise ValueError(f'joint_state 缺少 {self.name}')
        q = finite_number(positions[self.name], f'joint_state.{self.name}')
        if self.kind == 'prismatic':
            motion = _transform(self.axis * q, np.eye(3))
        else:
            x, y, z = self.axis
            cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            # Rodrigues 公式：在关节局部坐标系里绕 axis 转动。
            rotation = np.eye(3) + math.sin(q) * cross + (1 - math.cos(q)) * (cross @ cross)
            motion = _transform([0, 0, 0], rotation)
        return self.origin @ motion


@dataclass(frozen=True)
class BajieChain:
    joints: tuple[_Joint, ...]

    @classmethod
    def from_urdf(cls, path, base='base_footprint', tip='objects_link'):
        """从末端沿父关节回溯，再倒序得到底盘到夹爪的链。"""
        root = ET.parse(path).getroot()
        links = {link.attrib['name'] for link in root.findall('link')}
        if root.tag != 'robot' or not {base, tip} <= links:
            raise ValueError(f'请提供含 {base} 和 {tip} 的已展开 URDF，而非 Xacro')
        by_child = {}
        for joint in root.findall('joint'):
            child = joint.find('child').attrib['link']
            if child in by_child:
                raise ValueError(f'URDF 连杆 {child} 有多个父关节')
            by_child[child] = joint
        chain, visited = [], set()
        cursor = tip
        while cursor != base:
            if cursor in visited or cursor not in by_child:
                raise ValueError(f'URDF 中不存在有效的 {base} → {tip} 关节链')
            visited.add(cursor)
            joint = by_child[cursor]
            chain.append(_Joint.from_xml(joint))
            cursor = joint.find('parent').attrib['link']
        return cls(tuple(reversed(chain)))

    def forward(self, positions):
        result = np.eye(4)
        for joint in self.joints:
            result = result @ joint.at(positions)
        return result
