"""Depth association checks without neural dependencies, ROS or hardware."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
    'patrol_nav_ws/src/lite3_patrol_nav'))
from lite3_patrol_nav.learned_matching import depth_correspondences


class DepthAssociationTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[300., 0., 200.], [0., 300., 120.], [0., 0., 1.]])
        self.pixels = np.array([[100., 100.], [200., 100.]])
        self.points = np.array([[-1., -.2, 3.], [0., -.3, 4.]])

    def test_unprojects_depth_along_actual_correspondence_ray(self):
        p0 = np.array([[103., 101.], [198., 99.]])
        p1 = p0 + 10.
        objects, pixels = depth_correspondences(p0, p1, np.array([.9, .8]),
                                                self.pixels, self.points, self.k)
        self.assertEqual(len(objects), 2)
        np.testing.assert_allclose(objects[:, 2], [3., 4.])
        projection = objects[:, :2]/objects[:, 2:3]*300.+np.array([200., 120.])
        np.testing.assert_allclose(projection, p0)
        np.testing.assert_allclose(pixels, p1)

    def test_no_depth_association_beyond_four_pixels(self):
        objects, _ = depth_correspondences(np.array([[110., 100.]]),
            np.array([[120., 100.]]), np.array([.9]), self.pixels, self.points, self.k)
        self.assertEqual(len(objects), 0)

    def test_sloping_plane_depth_is_interpolated_at_matched_pixel(self):
        reference = np.array([[98., 98.], [102., 98.], [98., 102.], [102., 102.]])
        depths = 1.0/(0.2 + .01*(reference[:, 0]-100.) + .005*(reference[:, 1]-100.))
        points = np.column_stack(((reference[:, 0]-200.)*depths/300.,
                                  (reference[:, 1]-120.)*depths/300., depths))
        objects, _ = depth_correspondences(np.array([[101., 99.]]),
            np.array([[111., 99.]]), np.array([.9]), reference, points, self.k)
        self.assertEqual(len(objects), 1)
        self.assertAlmostEqual(objects[0, 2], 1.0/.205, places=5)

    def test_rejects_foreground_background_depth_discontinuity(self):
        reference = np.array([[100., 100.], [103., 100.]])
        objects, _ = depth_correspondences(np.array([[101., 100.]]),
            np.array([[110., 100.]]), np.array([.9]), reference, self.points, self.k)
        self.assertEqual(len(objects), 0)

    def test_duplicate_depth_landmark_has_only_one_vote(self):
        objects, _ = depth_correspondences(np.array([[100., 100.], [101., 100.]]),
            np.array([[110., 100.], [111., 100.]]), np.array([.8, .9]),
            self.pixels, self.points, self.k)
        self.assertEqual(len(objects), 1)

    def test_low_confidence_match_rejected(self):
        objects, _ = depth_correspondences(self.pixels, self.pixels,
            np.array([.1, .2]), self.pixels, self.points, self.k)
        self.assertEqual(len(objects), 0)


if __name__ == '__main__':
    unittest.main()
