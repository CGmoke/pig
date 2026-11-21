"""Pure input, planning and result helpers for the three official touch cases."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET
from itertools import product

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


def classify_contact(a, b, toy, fingers, moving_bodies):
    if contact_pair(a, b, toy, fingers):
        return 'target'
    if a in moving_bodies or b in moving_bodies:
        return 'blocked'
    return None


def is_surface_contact(separation, impulse):
    """Reject predictive contacts across a positive gap, even with impulse."""
    return (math.isfinite(separation) and separation <= 0.
            and math.isfinite(impulse) and impulse > 1e-6)


class TouchMiss(RuntimeError):
    """Expected adaptive planning or contact failure; eligible for a clean retry."""


class RecoveredTouchMiss(TouchMiss):
    """A miss with no outbound motion or a verified return to the initial pose."""


def run_independent_cases(cases, attempt):
    rows, misses = [], []
    for case in cases:
        try:
            rows.append(attempt(case))
        except RecoveredTouchMiss as exc:
            misses.append((case, exc))
    return rows, misses


class ReturnFailure(RuntimeError):
    """Halt the sequence; a verified touch is not permission to skip homing."""


def execute_return(initial, visited, dt, step, *, settle_frames=12, max_hold_frames=180):
    """Retrace measured outbound poses; hold home until all joints stop moving.

    step sends one setpoint and returns measured positions, velocities, target
    contact and non-target contacts for that frame. No position reset is used.
    """
    release_frames = 0
    released = False
    position_tol = np.full(len(JOINTS), .01)
    position_tol[0] = .002  # Lift is prismatic (metres); other joints use radians.

    def checked_step(target):
        nonlocal release_frames, released
        positions, velocities, touching, blocked = step(target)
        if blocked:
            raise ReturnFailure(f'non-target contact during return: {blocked[0]}')
        if touching and released:
            raise ReturnFailure('fingertip touched the toy again during return')
        release_frames = 0 if touching else release_frames + 1
        released |= release_frames >= 4
        return positions, velocities, touching

    # The last measured pose is the starting point, not a future setpoint.
    # Retrace the near-contact portion more slowly before folding the arm back.
    previous = visited[-1]
    travel = 0.
    count = 0
    for target in (*reversed(visited[:-1]), initial):
        distance = float(np.max(np.abs(target - previous)))
        if distance == 0.:
            continue
        speed = .12 if travel < .12 else .40
        for setpoint in smooth_segment(previous, target, dt, speed):
            checked_step(setpoint)
            count += 1
        travel += distance
        previous = target

    consecutive = 0
    for _ in range(max_hold_frames):
        positions, velocities, touching = checked_step(initial)
        count += 1
        error = np.abs(positions - initial)
        speed = float(np.max(np.abs(velocities)))
        if not touching and np.all(error <= position_tol) and speed <= .02:
            consecutive += 1
            if consecutive >= settle_frames:
                return count, float(np.max(error)), speed
        else:
            consecutive = 0
    raise ReturnFailure('initial joint state did not settle before the return timeout')


@dataclass(frozen=True)
class Waypoint:
    phase: str
    positions: np.ndarray


@dataclass(frozen=True)
class TouchPose:
    label: str
    rotation: np.ndarray
    hover: np.ndarray
    approach: np.ndarray
    touch: np.ndarray
    open_angle: float
    touch_angle: float


def check_touch(phase, contacts, blocked):
    if blocked:
        raise TouchMiss(f'non-target contact: {blocked[0]}')
    if contacts and phase != 'touch':
        raise TouchMiss(f'early toy contact in {phase}')
    return bool(contacts)


def execute_adaptive(initial, waypoints, dt, observe_step, wrist_settled=None):
    """One observation per commanded step; never issue another step after a hit."""
    previous = initial
    for waypoint in waypoints:
        if waypoint.phase == 'touch' and wrist_settled is not None:
            # Hold the open terminal pose until measured motion has stopped.
            consecutive = 0
            for _ in range(90):
                observe_step(previous, 'approach')
                consecutive = consecutive + 1 if wrist_settled(previous) else 0
                if consecutive == 4:
                    break
            else:
                raise TouchMiss('open wrist did not settle before closing')
        speed = {'align': .45, 'open': .20, 'approach': .25, 'touch': .10}[waypoint.phase]
        for positions in smooth_segment(previous, waypoint.positions, dt, speed):
            result = observe_step(positions, waypoint.phase)
            if result is not None:
                return result
        previous = waypoint.positions
    # A final stationary command allows the bridge/drives to catch up; no further closing.
    for _ in range(12):
        result = observe_step(previous, 'touch')
        if result is not None:
            return result
    raise TouchMiss('no verified fingertip contact')


def run_strategy(strategy, attempt, report):
    if strategy != 'adaptive-fallback':
        return attempt(strategy)
    try:
        return attempt('adaptive')
    except TouchMiss as exc:
        report(f'adaptive miss ({exc}); reset scene before baseline')
    return attempt('baseline')


_BOX_SIGNS = np.array(list(product((-1., 1.), repeat=3)))


def _transform_points(pose, points):
    return points @ pose[:3, :3].T + pose[:3, 3]


class GripperGeometry:
    """Collision box corners in the URDF, with the distal box last on each finger."""

    def __init__(self, urdf):
        root = ET.parse(urdf).getroot()
        links = {link.get('name'): link for link in root.findall('link')}
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        self.tool = rigid_inverse(BajieChain.from_urdf(
            urdf, base='gripper_link', tip='objects_link').forward({}))
        self.fingers = []
        for i, side in enumerate(('left', 'right'), 1):
            link = f'gripper_{side}_link'
            joint = joints[f'arm_finger_joint{i}']
            origin = self.origin(joint.find('origin'))
            axis = np.array(joint.find('axis').get('xyz').split(), dtype=float)
            self.fingers.append((self.tool @ origin, axis / np.linalg.norm(axis), self.boxes(links[link])))
        self.palm = _transform_points(self.tool, self.boxes(links['gripper_link']).reshape(-1, 3))

    @staticmethod
    def origin(origin):
        return pose_to_matrix({key: list(map(float, (origin.get(key, '0 0 0') if origin is not None
                                                     else '0 0 0').split())) for key in ('xyz', 'rpy')})

    @staticmethod
    def boxes(link):
        boxes = []
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                continue
            half = np.array(box.get('size').split(), dtype=float) / 2
            boxes.append(_transform_points(GripperGeometry.origin(collision.find('origin')), _BOX_SIGNS * half))
        if not boxes:
            raise TouchMiss(f'no URDF collision boxes for {link.get("name")}')
        return np.array(sorted(boxes, key=lambda b: b[:, 0].max()))

    def at(self, angle):
        return np.array([self.finger_points(side, angle, boxes)
                         for side, (_, _, boxes) in enumerate(self.fingers)])

    def finger_points(self, side, angle, points):
        from scipy.spatial.transform import Rotation

        # Official mimic is 1:1. Each joint has its own signed axis.
        origin, axis, _ = self.fingers[side]
        return _transform_points(origin, points @ Rotation.from_rotvec(axis * angle).as_matrix().T)


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
        self.urdf = urdf
        self.finger_min = max(float(limits[n].get('lower')) for n in JOINTS[-2:])
        self.finger_max = min(float(limits[n].get('upper')) for n in JOINTS[-2:])
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
        for seed in self.approach_seeds():
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

    def adaptive_poses(self, points, toy_rotation):
        """Propose in base coordinates; authored geometry ranks poses, PhysX verifies them."""
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 4
                or not np.isfinite(points).all() or np.ptp(points[:, 2]) < .004):
            raise TouchMiss('missing or degenerate toy collision geometry')
        grip = GripperGeometry(self.urdf)
        ceiling = float(points[:, 2].max())
        floor = float(points[:, 2].min())
        down = np.array([0., 0., -1.])
        radial = self.case.target[:3, 3].copy()
        radial[2] = 0
        axes = []
        for column in (0, 1):
            axis = toy_rotation[:, column].copy()
            axis[2] = 0
            length = np.linalg.norm(axis)
            if length > 1e-6:
                axes.append(axis / length)
        # For organic shapes the collision footprint is often not aligned with the prim axes.
        if self.case.name != 'rubik_cube':
            _, vectors = np.linalg.eigh(np.cov(points[:, :2], rowvar=False))
            axes.extend(np.r_[vectors[:, i], 0.] for i in (0, 1))
        axes.sort(key=lambda axis: np.ptp(points @ axis) if self.case.name == 'dinosaur'
                  else abs(axis @ radial))
        open_angle = self.finger_max - .004
        sweep = np.linspace(open_angle, self.finger_min + .004, 65)
        fingers_open = grip.at(open_angle)
        spans = []
        for angle in sweep:
            pads = grip.at(angle)[:, -1]
            spans.append(float(pads[0, :, 1].min() - pads[1, :, 1].max()))
        plans = []
        # Test opposite tool rolls as well: a geometrically identical alignment can
        # require a very different arm posture under the official joint limits.
        seen = []
        for axis in axes:
            if any(abs(axis @ prev) > .995 for prev in seen):
                continue
            seen.append(axis)
            for closing in (axis, -axis):
                tangent = np.cross(down, closing)
                rotation = np.column_stack((down, closing, tangent))
                u, v = points @ closing, points @ tangent
                vlo, vhi = float(v.min()), float(v.max())
                mid = (vlo + vhi) / 2
                if self.case.name != 'soft_toy':
                    # Estimate the width around the body, not the dinosaur's head or tail.
                    strip = np.abs(v - mid) <= max(.021, .23 * (vhi - vlo))
                    section = points[strip]
                    if len(section) >= 4:
                        left, right = float(u[strip].max()), float(u[strip].min())
                        width = left - right
                        local_top = float(section[:, 2].max())
                        # The dinosaur's upper contour tapers inward from the body width.
                        # Descend into the upper body and reserve a little extra closure.
                        dinosaur = self.case.name == 'dinosaur'
                        inset = .19 if dinosaur else .12
                        pad_z = local_top - np.clip(inset * (local_top - floor), .006, .016)
                        center = (left + right) / 2
                        overlap = .011 if dinosaur else .003
                        hit = next((i for i, span in enumerate(spans) if span <= width - overlap), None)
                        if hit is not None and hit > 1:
                            # Full open-finger boxes below the local top must clear
                            # the toy corridor throughout the descent, not just at the endpoint.
                            distal = grip.at(float(sweep[hit]))[:, -1]
                            goal = center * closing + mid * tangent
                            goal[2] = pad_z + float(distal[:, :, 0].mean())
                            safe = True
                            for finger in fingers_open:
                                for box in finger:
                                    depth = goal[2] - float(box[:, 0].max())
                                    a, b = mid + box[:, 2].min() - .002, mid + box[:, 2].max() + .002
                                    nearby = (v >= a) & (v <= b) & (points[:, 2] >= depth - .002)
                                    lo = center + box[:, 1].min() - .002
                                    hi = center + box[:, 1].max() + .002
                                    if np.any(nearby & (u >= lo) & (u <= hi)):
                                        safe = False
                            # Ensure the open distal pads actually flank the target.
                            if (safe and spans[0] >= width + .010
                                    and goal[2] - grip.palm[:, 0].max() >= ceiling + .008):
                                hover = goal.copy()
                                hover[2] = max(.25, ceiling + .10)
                                label = 'cube_side' if self.case.name == 'rubik_cube' else 'dinosaur_body'
                                plans.append(TouchPose(label, rotation, hover, goal, goal.copy(),
                                                       open_angle, float(sweep[hit])))
        if self.case.name != 'rubik_cube':
            # Wide toys needn't fit between both fingers. Aim one distal pad at
            # an upper-body patch; the small final descent is itself contact-gated.
            center = (points.min(axis=0) + points.max(axis=0)) / 2
            central = np.linalg.norm(points[:, :2] - center[:2], axis=1) <= .026
            patch = points[central] if np.count_nonzero(central) >= 3 else points
            upper = patch[patch[:, 2] >= np.quantile(patch[:, 2], .8)]
            opened = open_angle
            closed = float(np.clip(self.case.joints[JOINTS[-2]] - .075, self.finger_min + .004, opened))
            for axis in seen[:2]:
                for closing in (axis, -axis):
                    rotation = np.column_stack((down, closing, np.cross(down, closing)))
                    for side in (0, 1):
                        pads = grip.at(closed)
                        pad = pads[side, -1].mean(axis=0)
                        pad[0] = pads[side, -1, :, 0].max()
                        # Try the broad upper patch first, then move that pad toward
                        # the side that leaves the unused finger outside the toy.
                        inward = closing if side == 0 else -closing
                        width = np.ptp(points @ inward)
                        targets = [center - inward * width * fraction for fraction in (0., .32, .46)]
                        for aim in targets:
                            samples = upper if np.array_equal(aim, center) else points[
                                points[:, 2] >= ceiling - .012]
                            target = samples[np.argmin(np.linalg.norm(samples[:, :2] - aim[:2], axis=1))]
                            goal = target - rotation @ pad
                            goal[2] -= .002
                            projected = (points - goal) @ rotation
                            other = np.vstack([grip.at(float(a))[1-side] for a in np.linspace(opened, closed, 5)])
                            if any(np.any(np.all((projected >= box.min(axis=0) - .002)
                                                 & (projected <= box.max(axis=0) + .002), axis=1))
                                   for box in other):
                                continue
                            if goal[2] - grip.palm[:, 0].max() < ceiling + .006:
                                continue
                            hover = goal + np.array([0., 0., max(.25, ceiling + .10) - goal[2]])
                            approach = goal + np.array([0., 0., .010])
                            label = 'soft_upper' if self.case.name == 'soft_toy' else 'dinosaur_single'
                            plans.append(TouchPose(label, rotation, hover, approach, goal, opened, closed))
                            break
        if not plans:
            raise TouchMiss('no collision-geometry candidate with room for the gripper')
        return plans

    def surface_close_poses(self, points, toy_rotation, fingers):
        """Aim a tilted fingertip at an exposed shoulder, not into a concavity."""
        from scipy.spatial import cKDTree

        grip = GripperGeometry(self.urdf)
        tree = cKDTree(points)
        open_angle = self.finger_max - .004
        aim_angle = max(self.finger_min + .05, open_angle - .082)
        limit = aim_angle - .05
        up = np.array([0., 0., 1.])
        # The dinosaur exposes the side of its crest, the soft toy its upper rim.
        column, direction, preferred_side = ((0, 1, 0) if self.case.name == 'dinosaur'
                                              else (1, 1, 1))
        opened = [grip.finger_points(i, open_angle, p) for i, p in enumerate(fingers)]
        aimed = [grip.finger_points(i, aim_angle, p) for i, p in enumerate(fingers)]
        closed = [grip.finger_points(i, limit, p) for i, p in enumerate(fingers)]
        plans = []
        for col, sign in ((column, direction), (column, -direction),
                          (1-column, 1), (1-column, -1)):
            radial = sign * toy_rotation[:, col].copy()
            radial[2] = 0.
            if np.linalg.norm(radial) < 1e-6:
                continue
            radial /= np.linalg.norm(radial)
            for tilt in (15., 25.):
                beta = math.radians(tilt)
                x = -math.cos(beta) * up - math.sin(beta) * radial
                y = math.cos(beta) * radial - math.sin(beta) * up
                elevation = math.radians(55.)
                outward = math.cos(elevation) * radial + math.sin(elevation) * up
                patch = points[np.argmax(points @ outward)]
                for side in (preferred_side, 1-preferred_side):
                    closing = y if side == 0 else -y
                    rotation = np.column_stack((x, closing, np.cross(x, closing)))
                    normal = rotation.T @ outward
                    tip = aimed[side][np.argmin(aimed[side] @ normal)]
                    goal = patch - rotation @ tip
                    if tip[0] < -.006:  # Use a distal pad, never the finger root.
                        continue
                    # A supporting plane clears the selected open pad, so
                    # lowering the wrist cannot be the source of its contact.
                    projection = tip @ normal
                    if ((opened[side] @ normal).min() - projection < .003
                            or projection - (closed[side] @ normal).min() < .001):
                        continue
                    other = 1 - side
                    offside = np.concatenate((opened[other], aimed[other], closed[other], grip.palm))
                    offside = offside @ rotation.T + goal
                    # Leave room for the other finger and palm during approach
                    # and closure. This is a coarse filter; PhysX guards the path.
                    if min(tree.query(offside + dz * up)[0].min()
                           for dz in (0., .008, .016, .032, .064)) < .008:
                        continue
                    low_z = min((p @ rotation[2]).min() for p in (*opened, *closed, grip.palm))
                    if goal[2] + low_z < .006:
                        continue
                    hover = goal.copy()
                    hover[2] = max(.25, goal[2] + .10)
                    plans.append(TouchPose(f'{self.case.name}_surface_close', rotation,
                                           hover, goal, goal, open_angle, limit))
        if not plans:
            raise TouchMiss('no exposed surface with a clear open approach')
        return plans

    def adaptive_waypoints(self, points, toy_rotation, *, visible_points=None, finger_points=None):
        initial = np.array([self.case.joints[j] for j in JOINTS])
        last_error = None
        if self.case.name != 'rubik_cube' and visible_points is not None:
            poses = self.surface_close_poses(visible_points, toy_rotation, finger_points)
        else:
            poses = self.adaptive_poses(points, toy_rotation)
            if self.case.name == 'dinosaur':
                poses.sort(key=lambda pose: pose.label != 'dinosaur_single')
        for pose in poses:
            starts = []
            for seed in self.approach_seeds():
                try:
                    q = self.solve(pose.hover, pose.rotation, seed)
                    if not any(np.max(np.abs(q - prior)) < .02 for prior in starts):
                        starts.append(q)
                except ValueError as exc:
                    last_error = exc
            # Try the IK branch requiring the least initial joint motion first.
            for q in sorted(starts, key=lambda q: np.max(np.abs(q - self.seed))):
                try:
                    route = [Waypoint('align', self._positions(initial, q, initial[-2])),
                             Waypoint('open', self._positions(initial, q, pose.open_angle))]
                    # The arm reaches its final pose with fingers open. Only
                    # then move the fingers; no downward motion during closing.
                    count = max(1, math.ceil(np.linalg.norm(pose.touch - pose.hover) / .010))
                    for i in range(1, count + 1):
                        next_q = self.solve(pose.hover + (pose.touch - pose.hover) * (i/count),
                                            pose.rotation, q)
                        if np.max(np.abs(next_q - q)) > .35:
                            raise ValueError('IK branch jumped between adjacent poses')
                        q = next_q
                        route.append(Waypoint('approach', self._positions(initial, q, pose.open_angle)))
                    route.append(Waypoint('touch', self._positions(initial, q, pose.touch_angle)))
                    return pose.label, route
                except ValueError as exc:
                    last_error = exc
        raise TouchMiss(f'no continuous adaptive IK path: {last_error}')

    @staticmethod
    def _positions(initial, arm, angle):
        positions = initial.copy()
        positions[2:8] = arm
        positions[-2:] = angle
        return positions

    def approach_seeds(self):
        return self.seed, [0, 1, 2, 0, -1.5, 0], [0, 1, -1.4, 0, 1, 0], [0, -1, -1.5, 0, 1, 0]
