#!/usr/bin/env python3
"""Project obstacles above the saved local ground into a private 2-D costmap cloud.

Input is metric /aligned_points in map. Output preserves map X/Y but uses
synthetic Z=0.5 for obstacle marking ONLY; never use it for 3-D geometry/VOA.
Unknown ground keeps the return as an obstacle. No live plane fitting or
obstacle-map clearing is performed.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import yaml


class GroundModel:
    def __init__(self, map_dir):
        directory = Path(map_dir)
        summary = json.loads((directory / 'traversability_summary.json').read_text())
        config = yaml.safe_load((directory / 'patrol_map.yaml').read_text())
        self.resolution = float(summary['grid']['resolution_m'])
        self.x_min = float(summary['bounds']['x_min'])
        self.y_min = float(summary['bounds']['y_min'])
        if (not np.isclose(config['resolution'], self.resolution)
                or not np.allclose(config['origin'], [self.x_min, self.y_min, 0])):
            raise ValueError('ground evidence and navigation map geometry differ')
        with np.load(directory / 'traversability_evidence.npz', allow_pickle=False) as data:
            self.ground = data['classification_ground_z'].copy()
            self.support = (data['ground_model_seed']
                            & (data['ground_hits'] >= summary['parameters']['min_ground_observations'])
                            & np.isfinite(self.ground)
                            & np.isfinite(data['ground_std'])
                            & (data['ground_std'] <= summary['parameters']['max_ground_std']))
        self.height, self.width = self.ground.shape
        if (self.ground.shape != (summary['grid']['height'], summary['grid']['width'])
                or self.support.shape != self.ground.shape):
            raise ValueError('invalid ground raster shape')
        self.minimum = float(summary['parameters']['min_obstacle_height'])
        self.maximum = float(summary['parameters']['max_obstacle_height'])
        if not (0 < self.resolution <= .2 and 0 < self.minimum <= .15
                and self.maximum >= 1.2 and np.isfinite(self.maximum)):
            raise ValueError('unsupported obstacle-height limits')

    def project(self, xyz):
        xyz = np.asarray(xyz)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('expected Nx3 points')
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        # Check bounds before integer conversion; enormous finite coordinates
        # must not wrap around into trusted cells.
        inside = ((xyz[:, 0] >= self.x_min) & (xyz[:, 0] < self.x_min+self.width*self.resolution)
                  & (xyz[:, 1] >= self.y_min) & (xyz[:, 1] < self.y_min+self.height*self.resolution))
        indices = np.flatnonzero(inside)
        col = np.floor((xyz[indices, 0]-self.x_min)/self.resolution).astype(int)
        row = self.height-1-np.floor((xyz[indices, 1]-self.y_min)/self.resolution).astype(int)
        trusted = self.support[row, col]
        keep = np.ones(len(xyz), dtype=bool)
        above_ground = xyz[indices, 2]-self.ground[row, col]
        keep[indices[trusted]] = ((above_ground[trusted] >= self.minimum)
                                  & (above_ground[trusted] <= self.maximum))
        output = np.array(xyz[keep], dtype='<f4', copy=True)
        output[:, 2] = .5
        return output, dict(input=len(xyz), kept=len(output),
                            unknown_ground=int(len(xyz)-np.count_nonzero(trusted)))


class ProjectionFreshness:
    """Jetson sensor stamps use CLOCK_MONOTONIC; preserve source time.

    No valid heartbeat on localization loss, stale/replayed/future input, or
    exceptions. This uses the existing velocity gate's 1.5 s sensor budget.
    A legitimately empty fresh cloud is valid, and never clears the static map.
    """
    def __init__(self, timeout=1.5):
        self.timeout = timeout
        self.ready = False
        self.ready_time = float('-inf')
        self.last_stamp = 0.0

    def update_ready(self, ready, now):
        self.ready = bool(ready)
        self.ready_time = now

    def accepts(self, header, now):
        stamp = header.stamp.sec + header.stamp.nanosec / 1e9
        return (self.ready and 0 <= now-self.ready_time <= self.timeout
                and header.frame_id == 'map' and header.stamp.sec >= 0
                and 0 <= header.stamp.nanosec < 1000000000
                and -.05 <= now-stamp <= self.timeout and stamp > self.last_stamp)

    def commit(self, header):
        self.last_stamp = header.stamp.sec + header.stamp.nanosec / 1e9


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    types = {7: 'f4', 8: 'f8'}
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                     'formats': [endian+types[fields[k].datatype] for k in ('x', 'y', 'z')],
                     'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                     'itemsize': msg.point_step})
    a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                   strides=(msg.row_step, msg.point_step))
    return np.column_stack([a[k].ravel() for k in ('x', 'y', 'z')])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    model = GroundModel(args.map_dir)
    if args.check:
        print('[PASS] Ground evidence: %dx%d; obstacle height %.2f..%.2f m above ground' %
              (model.width, model.height, model.minimum, model.maximum))
        return
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Bool, Header
    rclpy.init()
    node = rclpy.create_node('patrol_costmap_ground_filter')
    publisher = node.create_publisher(PointCloud2, '/patrol/costmap_obstacles', 1)
    heartbeat = node.create_publisher(Header, '/patrol/lidar_costmap_heartbeat', 1)
    freshness = ProjectionFreshness()
    ready_subscription = node.create_subscription(
        Bool, '/patrol/localization_ready',
        lambda msg: freshness.update_ready(msg.data, time.monotonic()),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    last_log = [0.0]
    def callback(msg):
        if not freshness.accepts(msg.header, time.monotonic()):
            return
        points, stats = model.project(cloud_xyz(msg))
        output = PointCloud2()
        output.header = msg.header
        output.height, output.width = 1, len(points)
        output.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                         for i, k in enumerate(('x', 'y', 'z'))]
        output.is_bigendian, output.is_dense = False, True
        output.point_step, output.row_step = 12, 12*len(points)
        output.data = points.tobytes()
        # Recheck after computation: overload cannot make old evidence fresh.
        if not freshness.accepts(msg.header, time.monotonic()):
            return
        publisher.publish(output)
        freshness.commit(msg.header)
        heartbeat.publish(output.header)
        if time.monotonic()-last_log[0] >= 5:
            node.get_logger().info('ground-relative obstacle projection: '+json.dumps(stats))
            last_log[0] = time.monotonic()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscription = node.create_subscription(PointCloud2, '/aligned_points', callback, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
