"""Geometric regressions without ROS, transport or robot commands."""
import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('visual_matching', ROOT /
    'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/visual_matching.py')
matching = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matching)


class ReferenceSearchTests(unittest.TestCase):
    def test_rank_changes_cannot_starve_a_reference(self):
        rng = np.random.RandomState(7)
        frames = [{'id': i} for i in range(107)]
        tried, selected = set(), []
        for _ in range(54):
            ranked = [frames[i] for i in rng.permutation(len(frames))]
            selected.extend(f['id'] for f in matching.next_reference_batch(ranked, [], tried, 2))
        self.assertEqual(len(selected), 107)
        self.assertEqual(set(selected), set(range(107)))

    def test_completed_pass_can_retry_after_view_changes(self):
        frames = [{'id': i} for i in range(3)]
        tried = {0, 1, 2}
        self.assertEqual(matching.next_reference_batch(frames, [], tried, 2), frames[:2])
        self.assertEqual(tried, {0, 1})

    def test_successful_reference_remains_available_for_confirmation(self):
        frames = [{'id': i} for i in range(5)]
        tried = set(range(5))
        for _ in range(4):
            self.assertEqual(matching.next_reference_batch(frames, [3], tried, 1), [frames[3]])
        self.assertEqual(matching.next_reference_batch([], [], tried, 2), [])


class MatchingTests(unittest.TestCase):
    def setUp(self):
        cv2.setRNGSeed(7)
        cv2.setNumThreads(1)
        self.rng = np.random.RandomState(42)
        self.k = np.array([[302., 0., 213.], [0., 302., 129.], [0., 0., 1.]])
        self.objects = self.rng.uniform([-1., -.6, 2.], [1., .6, 6.], (100, 3))
        self.rvec = np.array([.04, -.08, .03])
        self.tvec = np.array([.15, -.06, .12])
        pixels, _ = cv2.projectPoints(self.objects, self.rvec, self.tvec, self.k, None)
        self.pixels = pixels[:, 0, :]

    def test_refines_pose_with_majority_outliers(self):
        pixels = self.pixels.copy() + self.rng.normal(0., .3, (100, 2))
        pixels[40:] = self.rng.uniform([0., 0.], [424., 240.], (60, 2))
        solution, detail = matching.solve_candidate(self.objects, pixels, self.k, 15, 4.)
        self.assertIsNotNone(solution, detail)
        self.assertGreaterEqual(solution['inliers'], 38)
        self.assertLess(np.linalg.norm(solution['transform'][:3, 3] - self.tvec), .03)
        self.assertLess(solution['reprojection'], 1.)

    def test_planar_road_points_remain_supported(self):
        objects = self.objects.copy()
        objects[:, 2] = 4.
        pixels, _ = cv2.projectPoints(objects, self.rvec, self.tvec, self.k, None)
        solution, detail = matching.solve_candidate(objects, pixels, self.k, 15, 4.)
        self.assertIsNotNone(solution, detail)
        self.assertGreaterEqual(solution['inliers'], 95)

    def test_unrelated_images_do_not_produce_a_pose(self):
        pixels = self.rng.uniform([0., 0.], [424., 240.], (100, 2))
        solution, detail = matching.solve_candidate(self.objects, pixels, self.k, 15, 4.)
        self.assertIsNone(solution)
        self.assertLess(detail['inliers'], 15)

    def test_duplicate_image_points_cannot_inflate_support(self):
        pixels = np.repeat(self.pixels[:3], [34, 33, 33], axis=0)
        solution, detail = matching.solve_candidate(self.objects, pixels, self.k, 15, 4.)
        self.assertIsNone(solution)
        self.assertEqual(detail['reason'], 'degenerate_points')

    def test_duplicate_matches_keep_only_closest_landmark(self):
        query = self.rng.randint(0, 256, (20, 32)).astype(np.uint8)
        reference = np.repeat(query[:4], 5, axis=0)
        matches = matching.unique_ratio_matches(cv2.BFMatcher(cv2.NORM_HAMMING),
                                                reference, query, .8)
        self.assertEqual(len(matches), 4)
        self.assertEqual(len({match.trainIdx for match in matches}), 4)

    def test_invalid_landmarks_rejected(self):
        self.objects[0, 0] = np.nan
        solution, detail = matching.solve_candidate(self.objects, self.pixels, self.k, 15, 4.)
        self.assertIsNone(solution)
        self.assertEqual(detail['reason'], 'invalid_points')


if __name__ == '__main__':
    unittest.main()
