"""Actual decoder/projector/runtime callback tests, with no robot or motion."""
import copy
import importlib.util
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import subprocess
import tempfile
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('voa_costmap', ROOT/'scripts/voa_costmap_bridge.py')
M = importlib.util.module_from_spec(spec)
spec.loader.exec_module(M)


def transform(yaw=0, pitch=0, x=0, y=0):
    return NS(translation=NS(x=x, y=y, z=0), rotation=NS(
        x=-math.sin(yaw/2)*math.sin(pitch/2), y=math.cos(yaw/2)*math.sin(pitch/2),
        z=math.sin(yaw/2)*math.cos(pitch/2), w=math.cos(yaw/2)*math.cos(pitch/2)))


def grid(data=None, order='F', start=(0, 0), offset=0, stamp=100.):
    data = np.zeros((4, 6)) if data is None else np.asarray(data)
    nx, ny = data.shape
    labels = ('column_index', 'row_index') if order == 'F' else ('row_index', 'column_index')
    sizes = (ny, nx) if order == 'F' else (nx, ny)
    layout = NS(dim=[NS(label=labels[0], size=sizes[0], stride=nx*ny),
                     NS(label=labels[1], size=sizes[1], stride=sizes[1])], data_offset=offset)
    # A circular-buffer fixture whose known world cells stay at the same place.
    storage = np.roll(data, start, axis=(0, 1))
    layer = NS(layout=layout, data=[-999.]*offset + storage.ravel(order=order).tolist())
    return NS(header=NS(frame_id='robot_foot_print', stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
              info=NS(resolution=.1, length_x=nx*.1, length_y=ny*.1,
                      pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
              layers=['elevation'], data=[layer], outer_start_index=start[0], inner_start_index=start[1])


class GeometryTests(unittest.TestCase):
    def test_matrix_orders_buffer_wrap_and_offset_preserve_asymmetric_obstacles(self):
        a = np.zeros((4, 6)); a[0, 1] = .3; a[3, 4] = 1.
        expected = [[.15, .15, .3], [-.15, -.15, 1.]]
        for order in ('F', 'C'):
            for start in ((0, 0), (3, 2)):
                for offset in (0, 2):
                    points, stats = M.grid_obstacles(grid(a, order, start, offset), .1)
                    self.assertEqual(stats['obstacles'], 2)
                    np.testing.assert_allclose(sorted(points.tolist()), sorted(expected), atol=1e-7)

    def test_only_finite_voa_obstacles_no_unknown_to_free_or_fake_wall(self):
        a = np.array([[0., .05, .099, .101], [np.nan, np.inf, -np.inf, 2.]])
        points, stats = M.grid_obstacles(grid(a), .1)
        self.assertEqual(len(points), 2)
        self.assertEqual(stats['unknown'], 3)
        self.assertTrue(np.isfinite(points).all())

    def test_translated_grid_center(self):
        a = np.zeros((4, 6)); a[0, 0] = .2
        msg = grid(a); msg.info.pose.position.x = 1.; msg.info.pose.position.y = -2.
        points, _ = M.grid_obstacles(msg, .1)
        np.testing.assert_allclose(points[0], [1.15, -1.75, .2])

    def test_rigid_projection_uses_height_and_indoor_pitch_before_synthetic_z(self):
        xyz = np.array([[1., .2, 1.]])
        points = M.project_to_map(xyz, transform(yaw=math.pi/2, x=2, y=3))
        np.testing.assert_allclose(points, [[1.8, 4., .5]])
        points = M.project_to_map(xyz, transform(pitch=-math.pi/10))
        self.assertAlmostEqual(float(points[0, 0]), math.cos(math.pi/10)-math.sin(math.pi/10), places=6)
        self.assertEqual(points[0, 2], .5)

    def test_no_obstacles_is_valid_empty_cloud(self):
        points, _ = M.grid_obstacles(grid(), .1)
        self.assertEqual(M.project_to_map(points, transform()).shape, (0, 3))

    def test_malformed_frame_geometry_layout_and_values_fail_closed(self):
        mutations = [lambda m: setattr(m.header, 'frame_id', 'camera_link'),
                     lambda m: setattr(m.info, 'resolution', 0),
                     lambda m: setattr(m.info.pose.orientation, 'z', .1),
                     lambda m: setattr(m, 'outer_start_index', 4),
                     lambda m: setattr(m.data[0].layout.dim[0], 'label', 'unknown'),
                     lambda m: setattr(m.data[0].layout.dim[1], 'stride', 1),
                     lambda m: m.data[0].data.pop(),
                     lambda m: setattr(m, 'layers', ['height'])]
        for mutate in mutations:
            msg = grid(); mutate(msg)
            with self.assertRaises(ValueError): M.grid_obstacles(msg, .1)
        bad = transform(); bad.rotation.w = 0
        with self.assertRaises(ValueError): M.project_to_map(np.zeros((1, 3)), bad)

    def test_sensor_clock_age_not_ros_wall_clock_and_not_relabelled(self):
        self.assertTrue(M.fresh(100., 100.4))
        self.assertFalse(M.fresh(100., 100.51))
        self.assertFalse(M.fresh(100.1, 100.))
        self.assertFalse(M.fresh(1793000000., 100.))


class Cloud:
    def __init__(self): self.header = NS()


class Field(NS):
    FLOAT32 = 7


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.outputs = {}
        def publisher(kind, topic, qos):
            self.outputs[topic] = []
            return NS(publish=lambda m: self.outputs[topic].append(copy.deepcopy(m)))
        node = NS(create_publisher=publisher, get_logger=lambda: NS(info=lambda s: None))
        self.tf_until = 100.
        self.tf_queries = []
        def lookup(target, source, stamp):
            self.tf_queries.append(stamp)
            if stamp > self.tf_until: raise ValueError('future TF')
            return NS(transform=transform(x=1.))
        self.bridge = M.Bridge(node, NS(lookup_transform=lookup), Cloud, Field, NS,
                               NS(from_msg=M.stamp_seconds), .1)
        self.clock = patch.object(M.time, 'monotonic', return_value=100.2)
        self.now = self.clock.start(); self.addCleanup(self.clock.stop)

    def test_newest_transformable_sample_empty_cloud_and_original_stamp(self):
        self.bridge.receive(grid(stamp=100.))
        self.bridge.receive(grid(stamp=100.1))
        self.bridge.publish()
        out = self.outputs[M.TOPIC][-1]
        self.assertEqual(out.width, 0)
        self.assertEqual(out.header.frame_id, 'map')
        self.assertEqual(M.stamp_seconds(out.header.stamp), 100.)
        self.assertEqual(self.tf_queries, [100.1, 100.])
        self.assertEqual(len(self.outputs[M.HEARTBEAT]), 1)
        self.bridge.publish()
        self.assertEqual(len(self.outputs[M.TOPIC]), 1)
        self.tf_until = 100.1
        self.bridge.publish()
        self.assertEqual(len(self.outputs[M.TOPIC]), 2)

    def test_stale_and_replayed_frames_do_not_refresh_health(self):
        self.bridge.receive(grid(stamp=99.))
        self.bridge.publish()
        self.assertFalse(self.outputs[M.HEARTBEAT])
        self.bridge.receive(grid(stamp=100.)); self.bridge.publish()
        self.now.return_value = 100.6
        self.bridge.receive(grid(stamp=100.)); self.bridge.publish()
        self.assertEqual(len(self.outputs[M.HEARTBEAT]), 1)

    def test_absent_tf_never_publishes_cloud_or_health(self):
        self.tf_until = 99.
        self.bridge.receive(grid()); self.bridge.publish()
        self.assertFalse(self.outputs[M.TOPIC])
        self.assertFalse(self.outputs[M.HEARTBEAT])

    def test_delayed_ndt_uses_exact_leg_motion_without_restamping(self):
        self.now.return_value = 100.3
        anchor = NS(header=NS(frame_id='map', stamp=NS(sec=100, nanosec=0)),
                    child_frame_id='ndt_registration',
                    pose=NS(pose=NS(position=NS(x=1., y=2., z=.3),
                                    orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        self.bridge.registration(anchor)
        def lookup(target, source, stamp):
            if target == 'map': raise ValueError('NDT not caught up')
            self.assertEqual((target, source), ('robot_odom','leg_base_link'))
            return NS(transform=transform(x=2.+(stamp-100.)))
        self.bridge.buffer.lookup_transform = lookup
        a = np.zeros((4,6)); a[0,0] = .3
        self.bridge.receive(grid(a, stamp=100.2)); self.bridge.publish()
        out = self.outputs[M.TOPIC][-1]
        np.testing.assert_allclose(np.frombuffer(out.data,dtype='<f4'),[1.35,2.25,.5],atol=1e-6)
        self.assertAlmostEqual(M.stamp_seconds(out.header.stamp),100.2)
        self.assertEqual(self.bridge.predictions,1)

    def test_compensation_requires_recent_anchor_and_both_leg_samples(self):
        self.now.return_value = 100.3
        anchor = NS(header=NS(frame_id='map', stamp=NS(sec=100,nanosec=0)),
                    child_frame_id='ndt_registration',
                    pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        self.bridge.registration(anchor)
        self.tf_until = 0
        self.bridge.receive(grid(stamp=100.2)); self.bridge.publish()
        self.assertFalse(self.outputs[M.HEARTBEAT])
        self.now.return_value = 100.8
        self.bridge.receive(grid(stamp=100.7)); self.bridge.publish()
        self.assertFalse(self.outputs[M.HEARTBEAT])

    def test_compensation_respects_rotation_and_rejects_implausible_motion(self):
        base=transform(yaw=math.pi/2,x=1,y=2)
        matrix=M.compensated_pose(base,transform(yaw=math.pi/2),transform(yaw=math.pi/2,y=.2))
        np.testing.assert_allclose(matrix[:3,3],[1,2.2,0],atol=1e-7)
        for bad in (transform(x=.6),transform(yaw=.5)):
            with self.assertRaises(ValueError): M.compensated_pose(base,transform(),bad)

    def test_rigid_compensation_with_height_tilt_avoids_general_inverse(self):
        anchor = transform(yaw=-.3, pitch=.2, x=4., y=-2.)
        leg0 = transform(yaw=.4, pitch=-.1, x=2., y=3.)
        leg1 = transform(yaw=.45, pitch=-.08, x=2.1, y=3.1)
        leg0.translation.z = .3
        leg1.translation.z = .35
        expected = M.transform_matrix(anchor) @ np.linalg.inv(M.transform_matrix(leg0)) @ M.transform_matrix(leg1)
        with patch.object(M.np.linalg, 'inv', side_effect=AssertionError('general inverse used')):
            actual = M.compensated_pose(anchor, leg0, leg1)
        np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_malformed_grid_cannot_emit_a_health_signal(self):
        msg = grid(); msg.header.frame_id = 'camera_link'
        self.bridge.receive(msg)
        with self.assertRaises(ValueError): self.bridge.publish()
        self.assertFalse(self.outputs[M.HEARTBEAT])

    def test_real_obstacle_xyz_serialization(self):
        a = np.zeros((4, 6)); a[0, 0] = .5
        self.bridge.receive(grid(a)); self.bridge.publish()
        out = self.outputs[M.TOPIC][-1]
        self.assertEqual((out.point_step, out.row_step, out.width), (12, 12, 1))
        np.testing.assert_allclose(np.frombuffer(out.data, dtype='<f4'), [1.15, .25, .5])


class LauncherTests(unittest.TestCase):
    def test_dynamic_launcher_requires_and_supervises_bridge_process(self):
        # Run the actual shell supervisor against temporary fake child programs.
        with tempfile.TemporaryDirectory(prefix='voa_launcher_') as directory:
            root = Path(directory)
            (root/'navigation').mkdir(); (root/'scripts').mkdir(); (root/'bin').mkdir()
            install = root/'voa_overlay_ws/install'; (install/'voa/lib/voa').mkdir(parents=True)
            (install/'setup.bash').write_text('true\n')
            (root/'navigation/patrol_env.sh').write_text('true\n')
            (root/'bin/systemctl').write_text('#!/bin/sh\nexit 3\n')
            (root/'bin/systemctl').chmod(0o755)
            sleeping = 'import os,time\nprint("CHILD_PID="+str(os.getpid()),flush=True)\ntime.sleep(30)\n'
            (root/'scripts/voa_camera_tf.py').write_text(sleeping)
            (root/'scripts/voa_costmap_bridge.py').write_text(
                'import time\nprint("BRIDGE_STARTED",flush=True)\ntime.sleep(.2)\nraise SystemExit(7)\n')
            voa = install/'voa/lib/voa/voa_composition'
            voa.write_text('#!/usr/bin/env python3\n'+sleeping); voa.chmod(0o755)
            launcher = root/'launcher.sh'
            launcher.write_text((ROOT/'navigation/start_vendor_voa.sh').read_text().replace(
                'PROJECT=/home/ysc/lite3_patrol_project', 'PROJECT='+directory))
            result = subprocess.run(['bash', str(launcher)], capture_output=True, text=True, timeout=5,
                env=dict(os.environ, PATH=str(root/'bin')+':'+os.environ['PATH'], LITE3_DYNAMIC_OBSTACLES='1'))
            self.assertEqual(result.returncode, 7, result.stderr)
            self.assertIn('BRIDGE_STARTED', result.stdout)
            for line in result.stdout.splitlines():
                if line.startswith('CHILD_PID='):
                    with self.assertRaises(ProcessLookupError): os.kill(int(line.split('=')[1]), 0)


if __name__ == '__main__':
    unittest.main()
