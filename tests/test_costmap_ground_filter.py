"""Ground removal must retain real obstacles on tilted/elevated maps."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ground_filter', ROOT/'scripts/costmap_ground_filter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GroundFilterTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        summary = dict(grid=dict(resolution_m=.1, height=4, width=4),
                       bounds=dict(x_min=0., y_min=0.),
                       parameters=dict(min_ground_observations=2, max_ground_std=.12,
                                       min_obstacle_height=.15, max_obstacle_height=1.2))
        (self.path/'traversability_summary.json').write_text(json.dumps(summary))
        (self.path/'patrol_map.yaml').write_text(yaml.safe_dump(dict(resolution=.1, origin=[0, 0, 0])))
        row, col = np.indices((4, 4))
        self.ground = .2*(col+.5)*.1 + .3*(3-row+.5)*.1 + .7
        np.savez(self.path/'traversability_evidence.npz', classification_ground_z=self.ground,
                 ground_model_seed=np.ones((4, 4), bool), ground_hits=np.full((4, 4), 10),
                 ground_std=np.zeros((4, 4)))
        self.model = module.GroundModel(self.path)

    def test_tilted_ground_removed_but_low_object_retained_at_same_xy(self):
        z = self.ground[3, 0]
        points = np.array([[.05, .05, z], [.05, .05, z+.08],
                           [.05, .05, z+.20], [.05, .05, z+1.0]])
        output, stats = self.model.project(points)
        self.assertEqual(stats['kept'], 2)
        np.testing.assert_allclose(output[:, :2], points[2:, :2])
        np.testing.assert_allclose(output[:, 2], .5)

    def test_raster_rows_are_inverted_to_world_y(self):
        points = [[.05, .35, self.ground[0, 0]+.08],
                  [.05, .05, self.ground[3, 0]+.20]]
        output, _ = self.model.project(points)
        self.assertEqual(len(output), 1)
        np.testing.assert_allclose(output[0, :2], [.05, .05])

    def test_unknown_unsupported_and_out_of_map_returns_are_obstacles(self):
        self.model.support[3, 0] = False
        output, stats = self.model.project([[.05, .05, .7], [-.01, .1, 0], [.4, .1, 0]])
        self.assertEqual(len(output), 3)
        self.assertEqual(stats['unknown_ground'], 3)

    def test_nonfinite_and_empty_points_do_not_wrap_into_map(self):
        output, _ = self.model.project([[float('nan'), .1, .7], [.1, .1, float('inf')]])
        self.assertEqual(output.shape, (0, 3))
        output, _ = self.model.project(np.empty((0, 3)))
        self.assertEqual(output.shape, (0, 3))

    def test_other_map_geometry_rejected(self):
        (self.path/'patrol_map.yaml').write_text(yaml.safe_dump(dict(resolution=.1, origin=[1, 0, 0])))
        with self.assertRaises(ValueError):
            module.GroundModel(self.path)

    def test_organized_cloud_row_padding_and_big_endian(self):
        from types import SimpleNamespace as NS
        dtype = np.dtype({'names': ['x','y','z'], 'formats': ['>f4']*3,
                          'offsets': [0,4,8], 'itemsize': 16})
        backing = bytearray(80)
        a = np.ndarray((2,2), dtype=dtype, buffer=backing, strides=(40,16))
        for i,k in enumerate(('x','y','z')):
            a[k] = np.arange(4).reshape(2,2)+i*10
        msg = NS(fields=[NS(name=k, offset=i*4, datatype=7) for i,k in enumerate(('x','y','z'))],
                 is_bigendian=True, point_step=16, row_step=40, height=2, width=2, data=backing)
        np.testing.assert_equal(module.cloud_xyz(msg), [[0,10,20],[1,11,21],[2,12,22],[3,13,23]])


if __name__ == '__main__':
    unittest.main()
