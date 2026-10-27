import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('calibration_audit', ROOT / 'scripts/audit_fast_livo_calibration.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_optical_coordinates_and_imu_chain():
    # A camera looking along body +X sees a body-forward point at optical +Z.
    optical = audit.rigid([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [1, 0, 0])
    lidar = audit.rigid(np.eye(3), [0, 0, 0])
    imu = audit.rigid(np.eye(3), [0, 0, .1])
    cl, ic = audit.compose(lidar, optical, imu)
    point_lidar = np.array([3, -1, -.5, 1])
    np.testing.assert_allclose(cl @ point_lidar, [1, .5, 2, 1])
    np.testing.assert_allclose(ic @ cl @ point_lidar, [3, -1, -.4, 1])


def test_factory_transformed_cloud_rejected():
    with pytest.raises(ValueError, match='zero driver extrinsics'):
        audit.require_raw_lidar({'lidar_configs': [{'extrinsic_parameter': dict(roll=0, pitch=18, yaw=0, x=187, y=0, z=400)}]})


def test_resolution_must_match_camera_info():
    camera = yaml.safe_load((ROOT / 'calibration/camera_pinhole_424x240.yaml').read_text())['/**']['ros__parameters']['camera']
    assert audit.camera_parameters(camera, (424, 240))['fx'] == camera['fx']
    with pytest.raises(ValueError, match='resolution'):
        audit.camera_parameters(camera, (640, 480))
    camera['d4'] = .01
    with pytest.raises(ValueError, match='four distortion'):
        audit.camera_parameters(camera, (424, 240))


def test_reflection_is_not_rotation():
    with pytest.raises(ValueError, match='determinant'):
        audit.rigid(np.diag([1, 1, -1]), [0, 0, 0])
