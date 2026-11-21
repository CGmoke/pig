import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import touch


class ReturnHomeTests(unittest.TestCase):
    def test_replays_only_visited_poses_before_finishing_at_home(self):
        home = np.zeros(10)
        hover = home.copy()
        hover[2] = .08
        approach = hover.copy()
        approach[3] = .03
        hit = approach.copy()
        hit[-2:] = -.012
        sent = []

        def step(q):
            sent.append(q.copy())
            return q, np.zeros(10), len(sent) <= 2, []

        touch.execute_return(home, [home, hover, approach, hit], 1/60, step)
        first_approach = next(i for i, q in enumerate(sent) if np.array_equal(q, approach))
        first_hover = next(i for i, q in enumerate(sent) if np.array_equal(q, hover))
        self.assertLess(first_approach, first_hover)
        for q in sent[:first_approach + 1]:
            np.testing.assert_array_equal(q[:8], hit[:8])
        np.testing.assert_array_equal(sent[-1], home)
        np.testing.assert_array_equal(hit[-2:], [-.012, -.012])
        deltas = np.diff(np.vstack([hit, *sent]), axis=0)
        self.assertLessEqual(np.max(np.abs(deltas)), .40/60 + 1e-12)
        self.assertLessEqual(np.max(np.abs(deltas[0])), .12/60 + 1e-12)

    def test_requires_consecutive_settled_feedback_including_velocity(self):
        home = np.zeros(10)
        calls = []

        def step(q):
            calls.append(q.copy())
            # Moving through the correct pose is not settled. First send some
            # in-tolerance readings, then break the streak with motion.
            velocity = np.full(10, .1 if len(calls) in (1, 5) else 0.)
            return home, velocity, False, []

        touch.execute_return(home, [home], 1/60, step, settle_frames=4, max_hold_frames=12)
        self.assertEqual(len(calls), 9)

    def test_mimic_finger_not_at_home_prevents_the_next_case(self):
        home = np.zeros(10)
        stuck = home.copy()
        stuck[-1] = .03
        with self.assertRaisesRegex(touch.ReturnFailure, 'not settle'):
            touch.execute_return(home, [home], 1/60,
                                 lambda q: (stuck, home, False, []), max_hold_frames=10)

    def test_return_aborts_on_non_target_contact_or_recontact(self):
        home = np.zeros(10)
        with self.assertRaisesRegex(touch.ReturnFailure, 'non-target contact'):
            touch.execute_return(home, [home], 1/60,
                                 lambda q: (home, home, False, ['obstacle']))

        flags = iter([True, False, False, False, False, True])
        with self.assertRaisesRegex(touch.ReturnFailure, 'touched the toy again'):
            touch.execute_return(home, [home], 1/60,
                                 lambda q: (home, home, next(flags), []))

    def test_cannot_finish_while_target_contact_is_still_present(self):
        home = np.zeros(10)
        with self.assertRaisesRegex(touch.ReturnFailure, 'not settle'):
            touch.execute_return(home, [home], 1/60,
                                 lambda q: (home, home, True, []), max_hold_frames=10)

    def test_failed_homing_is_not_a_touch_miss_or_permission_to_reset(self):
        attempts, notices = [], []

        def attempt(strategy):
            attempts.append(strategy)
            raise touch.ReturnFailure('return blocked')

        with self.assertRaisesRegex(touch.ReturnFailure, 'return blocked'):
            touch.run_strategy('adaptive-fallback', attempt, notices.append)
        self.assertEqual(attempts, ['adaptive'])
        self.assertEqual(notices, [])


if __name__ == '__main__':
    unittest.main()
