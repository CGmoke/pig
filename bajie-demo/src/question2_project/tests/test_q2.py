"""题二：题面基准、独立物理量校验与批处理契约。"""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from fk import BajieChain, pose_to_matrix, matrix_to_pose, rigid_inverse
from solve_q2 import solve_case

PROJECT = Path(__file__).resolve().parents[1]
INPUT = PROJECT.parents[1] / 'inputs/q2_demo_input.jsonl'
URDF = Path(os.environ.get('Q2_URDF', PROJECT.parents[2] / 'generated/bajie_robot.urdf'))


class QuestionTwoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = BajieChain.from_urdf(URDF)
        cls.example = json.loads(INPUT.read_text(encoding='utf-8'))

    def test_official_zero_case(self):
        result = solve_case(self.model, self.example)
        self.assertEqual(result['id'], 'demo_zero')
        expectations = {
            'objects_link_in_base_footprint': ([1.103715, -0.062685, 0.629136], 1),
            'scene_object_in_objects_link': ([-0.603441, -0.437315, -0.329638], -1),
        }
        self.assertEqual(set(result), {'id', *expectations})
        for field, (position, sign) in expectations.items():
            pose = result[field]
            self.assertEqual(set(pose), {'xyz', 'rpy', 'quat_xyzw'})
            np.testing.assert_allclose(pose['xyz'], position, atol=5e-7, rtol=0)
            np.testing.assert_allclose(pose['rpy'], [0, sign * 0.000832, 0], atol=5e-7, rtol=0)
            np.testing.assert_allclose(pose['quat_xyzw'], [0, sign * 0.000416, 0, 1],
                                       atol=5e-7, rtol=0)

    def test_lift_adds_only_base_z_and_head_fingers_are_other_branches(self):
        zero = self.example['joint_state']
        nominal = self.model.forward(zero)
        lifted = self.model.forward(dict(zero, lift_joint=0.1))
        np.testing.assert_allclose(lifted[:3, 3] - nominal[:3, 3], [0, 0, 0.1], atol=1e-12)
        np.testing.assert_allclose(lifted[:3, :3], nominal[:3, :3], atol=1e-12)
        np.testing.assert_array_equal(self.model.forward(dict(
            zero, pitch_joint=0.6, arm_finger_joint1=0.2, arm_finger_joint2=0.2)), nominal)

    def test_rotated_object_reconstructs_in_base_for_bent_arm(self):
        case = deepcopy(self.example)
        case['joint_state'].update(lift_joint=0.12, joint1=0.4, joint2=0.5,
                                   joint3=-0.2, joint4=0.3, joint5=-0.4, joint6=0.15)
        given = case['scene_object_in_base_footprint']
        given['rpy'] = [0.2, -0.3, 0.5]
        result = solve_case(self.model, case)
        hand = pose_to_matrix(result['objects_link_in_base_footprint'])
        obj = pose_to_matrix(result['scene_object_in_objects_link'])
        np.testing.assert_allclose(hand @ obj, pose_to_matrix(given), atol=1e-12)
        np.testing.assert_allclose(hand @ rigid_inverse(hand), np.eye(4), atol=1e-12)

    def test_pose_handles_gimbal_lock_and_xyzw_convention(self):
        for pitch in (math.pi / 2, -math.pi / 2):
            matrix = pose_to_matrix({'xyz': [0, 0, 0], 'rpy': [0.7, pitch, -0.4]})
            pose = matrix_to_pose(matrix)
            np.testing.assert_allclose(pose_to_matrix(pose), matrix, atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(pose['quat_xyzw']), 1.0)
        pose = matrix_to_pose(pose_to_matrix({'xyz': [0, 0, 0], 'rpy': [0, 0, math.pi / 2]}))
        np.testing.assert_allclose(pose['quat_xyzw'], [0, 0, 2**-0.5, 2**-0.5], atol=1e-12)

    def test_exact_half_turn_about_each_axis(self):
        for axis in range(3):
            with self.subTest(axis=axis):
                rpy = [0.0] * 3
                rpy[axis] = math.pi
                pose = matrix_to_pose(pose_to_matrix({'xyz': [0, 0, 0], 'rpy': rpy}))
                expected = np.zeros(4)
                expected[axis] = 1.0
                np.testing.assert_allclose(pose['quat_xyzw'], expected, atol=1e-12)

    def test_near_gimbal_lock_keeps_both_rotation_encodings_accurate(self):
        hand = self.model.forward(self.example['joint_state'])
        for sign in (-1, 1):
            for offset in (2e-15, 1e-13, 1e-9):
                with self.subTest(sign=sign, offset=offset):
                    target = pose_to_matrix({'xyz': [0, 0, 0],
                                             'rpy': [0.7, sign * (math.pi / 2 - offset), -0.4]})
                    # 模拟物体先转到底盘系，再换回夹爪系时的舍入误差。
                    relative = rigid_inverse(hand) @ (hand @ target)
                    pose = matrix_to_pose(relative)
                    np.testing.assert_allclose(pose_to_matrix(pose), target, atol=1e-12, rtol=0)
                    x, y, z, w = pose['quat_xyzw']
                    quat_rotation = np.array([
                        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
                    ])
                    np.testing.assert_allclose(quat_rotation, target[:3, :3], atol=1e-12, rtol=0)

    def test_missing_joint_and_non_finite_pose_are_rejected(self):
        case = deepcopy(self.example)
        del case['joint_state']['pitch_joint']
        with self.assertRaisesRegex(ValueError, 'pitch_joint'):
            solve_case(self.model, case)
        case = deepcopy(self.example)
        case['scene_object_in_base_footprint']['rpy'][0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'rpy'):
            solve_case(self.model, case)

    def test_cli_preserves_order_and_ignores_blank_lines(self):
        with tempfile.TemporaryDirectory() as tmp:
            source, output = Path(tmp) / 'input.jsonl', Path(tmp) / 'output.jsonl'
            source.write_text(json.dumps(self.example) + '\n\n' +
                              json.dumps(dict(self.example, id='second')) + '\n')
            command = [sys.executable, str(PROJECT / 'solve_q2.py'), '--urdf', str(URDF),
                       '--input', str(source), '--output', str(output)]
            success = subprocess.run(command, capture_output=True, text=True, check=True)
            self.assertIn('2', success.stdout)
            self.assertEqual([json.loads(line)['id'] for line in output.read_text().splitlines()],
                             ['demo_zero', 'second'])
            before = output.read_bytes()
            source.write_text(json.dumps(self.example) + '\n{\n')
            failure = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(failure.returncode, 0)
            self.assertIn('input.jsonl:2', failure.stderr)
            self.assertEqual(output.read_bytes(), before)
