import importlib.util
from pathlib import Path
import sys
import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('render_projection', SCRIPTS/'render_projection_validation.py')
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


def test_projection_occlusion_and_behind_camera():
    points = np.array([[0., 0, 2], [0, 0, 4], [0, 0, -1], [10, 0, 1]])
    k = np.array([[10., 0, 5], [0, 10, 5], [0, 0, 1]])
    z = renderer.project(points, np.eye(4), k, np.zeros(5), (10, 10))
    assert z[5, 5] == 2
    assert (z>0).sum() == 1


def test_translation_maps_known_point_and_empty_cloud():
    k = np.array([[10., 0, 5], [0, 10, 5], [0, 0, 1]])
    transform = np.eye(4)
    transform[0, 3] = -1
    z = renderer.project(np.array([[1., 0, 2]]), transform, k, np.zeros(5), (10, 10))
    assert z[5,5] == 2
    assert renderer.residuals(z, z)['median_abs_depth_error_m'] == 0
    empty = renderer.project(np.empty((0,3)), transform, k, np.zeros(5), (10,10))
    assert renderer.residuals(empty, z)['median_abs_depth_error_m'] is None
