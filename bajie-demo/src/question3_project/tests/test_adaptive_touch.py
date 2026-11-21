import math
import os
import sys
import unittest
from pathlib import Path

import numpy as np

Q3 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Q3))
import touch

URDF = Path(os.environ.get('BAJIE_URDF', Q3.parents[2] / 'generated/bajie_robot.urdf'))


def box_points(lo, hi):
    grid = np.stack(np.meshgrid(*[np.linspace(a, b, 11) for a, b in zip(lo, hi)]), axis=-1)
    return grid.reshape(-1, 3)


def yaw(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class AdaptiveGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = touch.load_cases(Q3.parents[1] / 'inputs/q3_demo_input.jsonl')

    def setup_geometry(self, i, points, angle):
        case = self.cases[i]
        rot = yaw(angle)
        base_xy = case.target[:, 3].copy()[:3]
        base_xy[2] = 0
        planner = touch.Planner(URDF, case)
        return planner, points @ rot.T + base_xy, rot

    def test_cube_aligns_with_face_and_closes_without_moving_wrist(self):
        planner, points, rot = self.setup_geometry(0, box_points((-.028, -.028, 0), (.028, .028, .056)), .40)
        plan = planner.adaptive_poses(points, rot)[0]
        self.assertEqual(plan.label, 'cube_side')
        self.assertAlmostEqual(max(abs(plan.rotation[:, 1] @ rot[:, i]) for i in (0, 1)), 1)
        np.testing.assert_allclose(plan.approach, plan.touch)
        self.assertGreater(plan.open_angle, plan.touch_angle)
        self.assertGreater(plan.hover[2], points[:, 2].max() + .08)

    def test_dinosaur_prefers_narrow_body_axis_over_tall_head(self):
        body = box_points((-.065, -.023, .006), (.055, .023, .080))
        head = box_points((-.083, -.020, .062), (-.059, .020, .112))
        planner, points, rot = self.setup_geometry(1, np.vstack((body, head)), .30)
        plans = planner.adaptive_poses(points, rot)
        self.assertEqual(plans[0].label, 'dinosaur_body')
        self.assertGreater(abs(plans[0].rotation[:, 1] @ rot[:, 1]), .97)
        self.assertLess(plans[0].touch[2], .095)

    def test_dinosaur_execution_tries_single_pad_before_side_closing(self):
        body = box_points((-.065, -.023, .006), (.055, .023, .080))
        head = box_points((-.083, -.020, .062), (-.059, .020, .112))
        planner, points, rot = self.setup_geometry(1, np.vstack((body, head)), .30)
        self.assertEqual(planner.adaptive_waypoints(points, rot)[0], 'dinosaur_single')

    def test_wide_toy_uses_single_pad_and_short_descent(self):
        for i, label in ((1, 'dinosaur_single'), (2, 'soft_upper')):
            with self.subTest(toy=label):
                planner, points, rot = self.setup_geometry(i, box_points((-.081, -.075, .006), (.081, .075, .095)), .60)
                plan = planner.adaptive_poses(points, rot)[0]
                self.assertEqual(plan.label, label)
                self.assertGreater(plan.approach[2], plan.touch[2])
                self.assertGreater(plan.open_angle, plan.touch_angle)
                self.assertLessEqual(plan.open_angle - plan.touch_angle, .08 + 1e-9)

    def test_three_plans_solve_with_official_joint_limits_and_ordered_phases(self):
        for i, dims in enumerate(((.028, .028, .056), (.055, .023, .096), (.048, .037, .105))):
            with self.subTest(case=i):
                x, y, height = dims
                planner, points, rot = self.setup_geometry(i, box_points((-x, -y, .005), (x, y, height)), .15)
                label, trajectory = planner.adaptive_waypoints(points, rot)
                self.assertIn(label, ('cube_side', 'dinosaur_body', 'dinosaur_single', 'soft_upper'))
                self.assertEqual(trajectory[0].phase, 'align')
                self.assertIn('approach', [step.phase for step in trajectory])
                self.assertEqual(trajectory[-1].phase, 'touch')
                for waypoint in trajectory:
                    self.assertTrue(np.all(waypoint.positions[2:8] >= planner.lower))
                    self.assertTrue(np.all(waypoint.positions[2:8] <= planner.upper))
                    self.assertEqual(waypoint.positions[-2], waypoint.positions[-1])

    def test_missing_geometry_is_a_planning_miss(self):
        planner = touch.Planner(URDF, self.cases[0])
        with self.assertRaises(touch.TouchMiss):
            planner.adaptive_poses(np.empty((0, 3)), np.eye(3))


class AdaptiveExecutionTests(unittest.TestCase):
    def test_target_requires_surface_contact_not_a_positive_gap_with_impulse(self):
        # Previous GUI run reported ~0.09 mm and ~0.03 mm of separation
        # for the dinosaur and soft toy, despite a nonzero impulse.
        for gap in (9.154669533e-5, 2.511030470e-5, 1e-8):
            self.assertFalse(touch.is_surface_contact(gap, .0007))
        self.assertTrue(touch.is_surface_contact(0., .0007))
        self.assertTrue(touch.is_surface_contact(-.0001, .0007))
        self.assertFalse(touch.is_surface_contact(-.0001, 0.))
        self.assertFalse(touch.is_surface_contact(float('-inf'), .0007))
        self.assertFalse(touch.is_surface_contact(0., float('inf')))

    def test_contact_returns_same_step_without_further_closing(self):
        init = np.zeros(10)
        target = init.copy()
        target[-2:] = .03
        calls = []
        def step(q, phase):
            calls.append((q.copy(), phase))
            return {'id': 'contact'} if len(calls) == 2 else None

        result = touch.execute_adaptive(init, [touch.Waypoint('touch', target)], 1/60, step)
        self.assertEqual(result, {'id': 'contact'})
        self.assertEqual(len(calls), 2)
        self.assertLess(calls[-1][0][-1], .03)

    def test_late_contact_uses_stationary_holds_and_no_contact_is_a_miss(self):
        init = np.zeros(10)
        calls = []
        def step(q, phase):
            calls.append((q.copy(), phase))
            return 'hit' if len(calls) == 3 else None

        trajectory = [touch.Waypoint('touch', init)]
        self.assertEqual(touch.execute_adaptive(init, trajectory, 1/60, step), 'hit')
        for q, phase in calls:
            np.testing.assert_array_equal(q, init)
            self.assertEqual(phase, 'touch')
        with self.assertRaises(touch.TouchMiss):
            touch.execute_adaptive(init, trajectory, 1/60, lambda q, phase: None)

    def test_only_target_fingertip_in_touch_phase_succeeds(self):
        self.assertFalse(touch.check_touch('touch', [], []))
        self.assertTrue(touch.check_touch('touch', ['contact'], []))
        with self.assertRaises(touch.TouchMiss):
            touch.check_touch('approach', ['early'], [])
        with self.assertRaises(touch.TouchMiss):
            touch.check_touch('touch', ['contact'], [('arm', 'obstacle')])

    def test_contact_classification_distinguishes_robot_motion_from_toy_ground(self):
        fingers = ('left', 'right')
        moving = {'left', 'right', 'wrist', 'palm'}
        for a, b, result in (('left', 'toy', 'target'), ('toy', 'wrist', 'blocked'),
                             ('right', 'toy2', 'blocked'), ('ground', 'toy', None),
                             ('wrist', 'base', 'blocked'), ('base', 'ground', None)):
            with self.subTest(a=a, b=b):
                self.assertEqual(touch.classify_contact(a, b, 'toy', fingers, moving), result)
                self.assertEqual(touch.classify_contact(b, a, 'toy', fingers, moving), result)

    def test_only_expected_miss_triggers_a_new_baseline_attempt(self):
        calls, notices = [], []
        def attempt(mode):
            calls.append(mode)
            if mode == 'adaptive':
                raise touch.TouchMiss('no target contact')
            return 'baseline result'

        self.assertEqual(touch.run_strategy('adaptive-fallback', attempt, notices.append), 'baseline result')
        self.assertEqual(calls, ['adaptive', 'baseline'])
        self.assertIn('reset', notices[0])
        with self.assertRaises(touch.TouchMiss):
            touch.run_strategy('adaptive', attempt, notices.append)
        def broken(mode):
            raise RuntimeError('Bridge disconnected')
        with self.assertRaisesRegex(RuntimeError, 'Bridge disconnected'):
            touch.run_strategy('adaptive-fallback', broken, notices.append)


if __name__ == '__main__':
    unittest.main()
