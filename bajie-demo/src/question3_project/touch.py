"""Pure input, planning and result helpers for the three official touch cases."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'question2_project'))
from fk import BajieChain, finite_number, matrix_to_pose, pose_to_matrix, rigid_inverse

ARM = tuple(f'joint{i}' for i in range(1, 7))
JOINTS = ('lift_joint', 'pitch_joint', *ARM, 'arm_finger_joint1', 'arm_finger_joint2')
TOYS = {'rubik_cube': 'Rubik', 'dinosaur': 'toy_yellow_dragon', 'soft_toy': 'spiderman_grasp'}


def finger_pad_offset(urdf, open_angle):
    """左手指尖垫在 objects_link 坐标系中的位置：+X 向下、+Y 侧向、+Z 径向。"""
    root = ET.parse(urdf).getroot()
    distal = None
    for link in root.findall('link'):
        if link.attrib['name'] != 'gripper_left_link':
            continue
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                continue
            origin = collision.find('origin')
            xyz = [float(v) for v in
                   (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            half_x = float(box.attrib['size'].split()[0]) / 2
            candidate = (xyz[0] + half_x, xyz[1], 0.0)
            if distal is None or candidate[0] > distal[0]:
                distal = candidate
    if distal is None:
        raise ValueError('官方 URDF 缺少 gripper_left_link 的 box 碰撞几何')
    to_object = BajieChain.from_urdf(urdf, base='gripper_link', tip='objects_link').forward({})
    to_finger = BajieChain.from_urdf(urdf, base='gripper_link', tip='gripper_left_link').forward(
        {'arm_finger_joint1': open_angle})
    return (np.linalg.inv(to_object) @ to_finger @ np.r_[distal, 1.0])[:3]


@dataclass(frozen=True)
class Case:
    id: str
    name: str
    joints: dict
    base: np.ndarray
    target: np.ndarray


def load_cases(path):
    cases = []
    for number, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        try:
            row = json.loads(line)
            ident, name, joints = row['id'], row['name'], row['joint_state']
            if not isinstance(ident, str) or not ident or name not in TOYS or set(joints) != set(JOINTS):
                raise ValueError('invalid id, name, or joint names')
            joints = {j: finite_number(joints[j], j) for j in JOINTS}
            base = pose_to_matrix(row['base_footprint_in_world'])
            obj = pose_to_matrix(row['scene_object_in_world'])
        except (KeyError, ValueError, TypeError) as exc:
            raise ValueError(f'{path}:{number}: {exc}') from exc
        cases.append(Case(ident, name, joints, base, rigid_inverse(base) @ obj))
    if len(cases) != 3 or len({c.id for c in cases}) != 3 or {c.name for c in cases} != TOYS.keys():
        raise ValueError('Expected exactly one official case per toy, with distinct IDs')
    return cases


def answer(case, joints, base, hand, obj):
    return {'id': case.id, 'final_joint_state': {j: finite_number(float(joints[j]), j) for j in JOINTS},
            'objects_link_in_base_footprint': matrix_to_pose(rigid_inverse(base) @ hand),
            'scene_object_in_objects_link': matrix_to_pose(rigid_inverse(hand) @ obj)}


def write_answers(path, cases, rows):
    if len(cases) != 3 or [r['id'] for r in rows] != [c.id for c in cases]:
        raise ValueError('All three confirmed contacts are required to write the answer')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as f:
            tmp = Path(f.name)
            for row in rows:
                f.write(json.dumps(row, allow_nan=False) + '\n')
        tmp.replace(path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


def smooth_segment(start, stop, dt, speed=.5):
    delta = stop - start
    steps = max(1, math.ceil(1.5 * np.max(np.abs(delta)) / speed / dt))
    for i in range(1, steps + 1):
        t = i / steps
        yield start + (3 * t * t - 2 * t**3) * delta


def contact_pair(a, b, toy, fingers):
    return (a == toy and b in fingers) or (b == toy and a in fingers)


class Planner:
    """Follow a down-facing gripper pose through incremental IK waypoints."""

    def __init__(self, urdf, case):
        self.case = case
        self.chain = BajieChain.from_urdf(urdf)
        limits = {j.attrib['name']: j.find('limit') for j in ET.parse(urdf).findall('joint')}
        self.lower = np.array([float(limits[n].attrib['lower']) for n in ARM])
        self.upper = np.array([float(limits[n].attrib['upper']) for n in ARM])
        self.seed = np.array([case.joints[n] for n in ARM])
        for n in JOINTS:
            limit = limits[n].attrib
            if not float(limit['lower']) - 1e-6 <= case.joints[n] <= float(limit['upper']) + 1e-6:
                raise ValueError(f'Initial value out of URDF limits: {n}')
        self.pad = finger_pad_offset(urdf, case.joints['arm_finger_joint1'])
        self.pad_down = float(self.pad[0])
        self.rotation = np.eye(3)
        self.hand_xy = np.zeros(2)

    def forward(self, q):
        return self.chain.forward({**self.case.joints, **dict(zip(ARM, map(float, q)))})

    def solve(self, point, rotation, seed):
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation

        def residual(q):
            pose = self.forward(q)
            return np.r_[pose[:3, 3] - point,
                         .25 * Rotation.from_matrix(rotation.T @ pose[:3, :3]).as_rotvec()]

        fit = least_squares(residual, np.clip(seed, self.lower, self.upper),
                            bounds=(self.lower, self.upper), max_nfev=100, gtol=1e-8)
        if np.linalg.norm(fit.fun) > .004:
            raise ValueError(f'IK could not reach {point.tolist()} (error={np.linalg.norm(fit.fun):.4f})')
        return fit.x

    def descent(self, point, top=.25, bottom=None, step=.004):
        # objects_link +X 指向指尖；把左手指尖垫对准 point，而不是只对准工具原点。
        radial = np.array([point[0], point[1], 0.])
        radial /= np.linalg.norm(radial)
        down = np.array([0., 0., -1.])
        self.rotation = np.column_stack((down, np.cross(radial, down), radial))
        offset = self.rotation @ self.pad
        self.hand_xy = np.array([point[0], point[1]]) - offset[:2]
        floor = max(.004, point[2]) if bottom is None else bottom
        initial = np.array([self.hand_xy[0], self.hand_xy[1], top])
        candidates = []
        for seed in (self.seed, [0, 1, 2, 0, -1.5, 0], [0, 1, -1.4, 0, 1, 0], [0, -1, -1.5, 0, 1, 0]):
            try:
                candidates.append(self.solve(initial, self.rotation, seed))
            except ValueError:
                continue
        if not candidates:
            raise ValueError(f'No approach IK solution for {self.case.id}')
        q = min(candidates, key=lambda x: np.linalg.norm(x - self.seed))
        yield q
        for z in np.linspace(top, floor, 1 + math.ceil((top - floor) / step))[1:]:
            q = self.solve(np.array([self.hand_xy[0], self.hand_xy[1], z]), self.rotation, q)
            yield q

    def press(self, q, depth=.003, step=.001):
        """接触后沿同一方向继续下压，逐毫米给出更深的关节解。"""
        z = float(self.forward(q)[2, 3])
        for _ in range(max(0, int(round(depth / step)))):
            z -= step
            try:
                q = self.solve(np.array([self.hand_xy[0], self.hand_xy[1], z]),
                               self.rotation, q)
            except ValueError:
                return
            yield q
