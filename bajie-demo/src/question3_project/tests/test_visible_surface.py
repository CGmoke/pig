import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from surface import meshes_touch


class VisibleSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.finger = np.array([[[0., 0., 0.], [.010, 0., 0.], [0., .010, 0.]]])

    def test_positive_gap_is_not_contact_even_with_overlapping_bounds(self):
        toy = self.finger + [.006, .006, 0.]
        self.assertFalse(meshes_touch(self.finger, toy))
        self.assertFalse(meshes_touch(self.finger, self.finger + [0., 0., 1e-7]))

    def test_accepts_touching_coplanar_and_crossing_triangles(self):
        self.assertTrue(meshes_touch(self.finger, self.finger + [.010, 0., 0.]))
        toy = np.array([[[.002, .002, -.001], [.002, .002, .001], [.003, .003, .001]]])
        self.assertTrue(meshes_touch(self.finger, toy))
        self.assertTrue(meshes_touch(toy, self.finger))
