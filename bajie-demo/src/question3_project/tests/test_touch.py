import sys
import unittest
from pathlib import Path

import numpy as np

Q3 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Q3))
from touch import load_cases, answer, smooth_segment, contact_pair, pose_to_matrix


class TouchTests(unittest.TestCase):
    def test_official_input_and_base_frame_target(self):
        cases = load_cases(Q3.parents[1] / 'inputs/q3_demo_input.jsonl')
        self.assertEqual([c.name for c in cases], ['rubik_cube', 'dinosaur', 'soft_toy'])
        np.testing.assert_allclose(cases[0].target[:3, 3], [0.53565, -0.33019, 0.02892], atol=1e-6)

    def test_answer_frames(self):
        cases = load_cases(Q3.parents[1] / 'inputs/q3_demo_input.jsonl')
        c = cases[0]
        hand = pose_to_matrix({'xyz': [12.66, 5.55, .08], 'rpy': [0, 0, .4]})
        obj = c.base @ c.target
        row = answer(c, c.joints, c.base, hand, obj)
        self.assertEqual(row['id'], c.id)
        self.assertEqual(row['final_joint_state'], c.joints)
        np.testing.assert_allclose(c.base @ pose_to_matrix(row['objects_link_in_base_footprint']), hand)
        np.testing.assert_allclose(hand @ pose_to_matrix(row['scene_object_in_objects_link']), obj)

    def test_smooth_segment(self):
        samples = np.array(list(smooth_segment(np.zeros(2), np.array([1., -1.]), 1/60, speed=.5)))
        np.testing.assert_allclose(samples[-1], [1., -1.])
        self.assertLessEqual(np.max(np.abs(np.diff(samples, axis=0))) * 60, .50001)

    def test_contact_requires_distinct_matching_actors(self):
        fingers = ('/robot/left', '/robot/right')
        self.assertTrue(contact_pair('/toy', '/robot/right', '/toy', fingers))
        self.assertFalse(contact_pair('/toy', '/robot/arm', '/toy', fingers))
        self.assertFalse(contact_pair('/toy2', '/robot/right', '/toy', fingers))
        self.assertFalse(contact_pair('/toy', '/ground', '/toy', fingers))


if __name__ == '__main__':
    unittest.main()
