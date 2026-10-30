#!/usr/bin/env python3
"""Project the private VOA elevation grid into Nav2's map-frame obstacle cloud.

Lite3-specific contract: VOA and the patched NDT both register points after
IMU roll/pitch leveling and the same 0.34751 m foot-height translation. NDT's
historically named map->base_link TF is the registration pose of THAT frame.
Apply it once to grid cells (including elevation), then discard map Z for 2-D
marking. Do not apply the camera mount or foot-height correction again.

Both source grid and NDT TF use the Jetson monotonic sensor clock. Preserve
that stamp for TF/STVL; never relabel old observations with ROS wall time.
If exact-time NDT is pending, advance a recent full NDT pose with timestamped
leg odometry (at both the anchor and grid times). Never reuse a latest pose
without motion compensation. No motor topics are published. Unknown elevation stays under VOA's stop rule;
it is not fabricated into a physical obstacle or a clearing observation.
"""
from collections import deque
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

TOPIC = '/patrol/voa/costmap_obstacles'
HEARTBEAT = '/patrol/voa/costmap_heartbeat'
MAX_AGE = .5
MAX_PREDICTION = .5


def stamp_seconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('invalid sensor stamp')
    return stamp.sec + stamp.nanosec / 1e9


def fresh(stamp, now):
    # Small controller/host timestamp skew is already present on this Lite3.
    return math.isfinite(stamp) and stamp > 0 and -.05 <= now - stamp <= MAX_AGE


def grid_obstacles(msg, minimum, include_unknown=False):
    """Decode grid_map Eigen layout and circular indices, not OccupancyGrid."""
    if msg.header.frame_id != 'robot_foot_print':
        raise ValueError('expected factory robot_foot_print elevation grid')
    if (len(msg.layers) != len(msg.data) or msg.layers.count('elevation') != 1
            or not math.isfinite(minimum) or minimum <= 0):
        raise ValueError('missing/invalid elevation layer or obstacle threshold')
    info = msg.info
    r, lx, ly = info.resolution, info.length_x, info.length_y
    pose = info.pose
    p, q = pose.position, pose.orientation
    if (not all(math.isfinite(v) for v in (r, lx, ly, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or not .01 <= r <= .2 or not 0 < lx <= 10 or not 0 < ly <= 10
            or max(abs(p.z), abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w)-1)) > 1e-6):
        raise ValueError('unsupported grid geometry/orientation')
    nx, ny = round(lx/r), round(ly/r)
    if nx <= 0 or ny <= 0 or not np.allclose([nx*r, ny*r], [lx, ly], atol=1e-6):
        raise ValueError('grid size is not an integral number of cells')
    layer = msg.data[msg.layers.index('elevation')]
    dims = layer.layout.dim
    if len(dims) != 2:
        raise ValueError('grid must have two dimensions')
    labels = tuple(d.label for d in dims)
    if labels == ('column_index', 'row_index'):
        sizes, order = (ny, nx), 'F'
    elif labels == ('row_index', 'column_index'):
        sizes, order = (nx, ny), 'C'
    else:
        raise ValueError('unknown grid matrix storage order')
    if (tuple(d.size for d in dims) != sizes
            or tuple(d.stride for d in dims) != (nx*ny, sizes[1])):
        raise ValueError('grid matrix size/stride mismatch')
    offset = layer.layout.data_offset
    if offset < 0 or len(layer.data) != offset + nx*ny:
        raise ValueError('grid matrix data size mismatch')
    elevation = np.asarray(layer.data, dtype=np.float32)[offset:].reshape((nx, ny), order=order)
    start = (msg.outer_start_index, msg.inner_start_index)
    if not (0 <= start[0] < nx and 0 <= start[1] < ny):
        raise ValueError('invalid circular grid start index')
    # The factory compares float elevation (promoted to double) strictly >.
    occupied = np.zeros(elevation.shape, dtype=bool)
    finite = np.isfinite(elevation)
    occupied[finite] = elevation[finite].astype(np.float64) > minimum
    # VOA rejects trajectories through unobserved cells. Expose those same
    # cells to the planner as conservative constraints, separately counted.
    # Zero below is a projection reference at the foot plane, not a measured
    # obstacle height or a claim that unknown space is free.
    if include_unknown:
        occupied |= ~finite
    row, col = np.nonzero(occupied)
    ix, iy = (row-start[0]) % nx, (col-start[1]) % ny
    xyz = np.column_stack((p.x + lx/2 - (ix+.5)*r,
                           p.y + ly/2 - (iy+.5)*r,
                           np.where(finite[row,col],elevation[row,col],0.)))
    return xyz, {'cells': nx*ny, 'obstacles': len(xyz),
                 'unknown_constraints': int(np.count_nonzero(~finite)) if include_unknown else 0,
                 'unknown': int(np.count_nonzero(~np.isfinite(elevation)))}


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite map transform')
    x, y, z, w = values[3:]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if abs(norm-1) > .01:
        raise ValueError('invalid map transform quaternion')
    x, y, z, w = (a/norm for a in (x, y, z, w))
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = values[:3]
    return matrix


def compensated_pose(anchor, leg_anchor, leg_grid):
    """T_map_registration(t0) * inverse(T_leg(t0)) * T_leg(t1)."""
    leg0, leg1 = transform_matrix(leg_anchor), transform_matrix(leg_grid)
    # These are rigid transforms: invert with R.T, without a general LAPACK
    # solve. Tiny 4x4 inverses were waking BLAS workers on the Jetson and
    # competing with the very TF/data callbacks this bridge depends on.
    delta = np.eye(4)
    delta[:3, :3] = leg0[:3, :3].T @ leg1[:3, :3]
    delta[:3, 3] = leg0[:3, :3].T @ (leg1[:3, 3] - leg0[:3, 3])
    angle = math.acos(float(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1)))
    if np.linalg.norm(delta[:3, 3]) > .5 or angle > .35:
        raise ValueError('leg motion exceeds bounded projection prediction')
    return transform_matrix(anchor) @ delta


def project_to_map(xyz, transform):
    matrix = transform if isinstance(transform, np.ndarray) else transform_matrix(transform)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('invalid projection matrix')
    points = np.asarray(xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('invalid obstacle points')
    output = points @ matrix[:3, :3].T + matrix[:3, 3]
    # Keep metric X/Y, including roll/pitch effects of the indoor registration
    # correction. Synthetic Z must be assigned AFTER the transform.
    output[:, 2] = .5
    return output.astype('<f4')


class Bridge:
    def __init__(self, node, buffer, cloud_type, field_type, header_type, time_type, minimum,
                 output_topic=TOPIC, heartbeat_topic=HEARTBEAT):
        self.node, self.buffer = node, buffer
        self.Cloud, self.Field, self.Header, self.Time = cloud_type, field_type, header_type, time_type
        self.minimum = minimum
        self.pending = deque(maxlen=16)
        self.last_received = self.last_published = 0.
        self.last_log = 0.
        self.announced = False
        self.anchors = deque(maxlen=12)
        self.predictions = 0
        self.publisher = node.create_publisher(cloud_type, output_topic, 1)
        self.heartbeat = node.create_publisher(header_type, heartbeat_topic, 1)

    def receive(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if fresh(stamp, time.monotonic()) and stamp > self.last_received:
            self.pending.append(msg)
            self.last_received = stamp

    def registration(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if (msg.header.frame_id != 'map' or msg.child_frame_id != 'ndt_registration'
                or not 0 <= time.monotonic()-stamp <= MAX_AGE
                or (self.anchors and stamp <= stamp_seconds(self.anchors[-1].header.stamp))):
            return
        p = msg.pose.pose
        try:
            transform_matrix(SimpleNamespace(translation=p.position, rotation=p.orientation))
        except (ValueError, TypeError, AttributeError):
            return
        self.anchors.append(msg)

    def projection_transform(self, msg):
        target_time = self.Time.from_msg(msg.header.stamp)
        try:
            return self.buffer.lookup_transform('map', 'base_link', target_time).transform
        except Exception:
            pass
        stamp = stamp_seconds(msg.header.stamp)
        # Both leg transforms must exist at their exact requested timestamps.
        # Missing/stale odometry never refreshes the obstacle heartbeat.
        for anchor in reversed(self.anchors):
            age = stamp-stamp_seconds(anchor.header.stamp)
            if not 0 <= age <= MAX_PREDICTION:
                continue
            leg0 = self.buffer.lookup_transform('robot_odom', 'leg_base_link', self.Time.from_msg(anchor.header.stamp))
            leg1 = self.buffer.lookup_transform('robot_odom', 'leg_base_link', target_time)
            pose = anchor.pose.pose
            result = compensated_pose(SimpleNamespace(translation=pose.position, rotation=pose.orientation),
                                      leg0.transform, leg1.transform)
            self.predictions += 1
            return result
        raise ValueError('no recent NDT anchor with timestamped leg odometry')

    def publish(self):
        now = time.monotonic()
        # NDT runs at 10 Hz. Keep a bounded backlog so a newer camera frame
        # does not continually replace the one waiting for interpolated TF.
        reason = 'waiting for fresh grid and matching NDT TF'
        for msg in reversed(self.pending):
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= self.last_published or not fresh(stamp, now):
                continue
            try:
                transform = self.projection_transform(msg)
            except Exception:
                continue  # Never fall back to latest pose for an older frame.
            try:
                xyz, stats = grid_obstacles(msg, self.minimum, include_unknown=True)
                points = project_to_map(xyz, transform)
            except (ValueError, TypeError, AttributeError, IndexError):
                continue
            output = self.Cloud()
            output.header.stamp = msg.header.stamp
            output.header.frame_id = 'map'
            output.height, output.width = 1, len(points)
            output.fields = [self.Field(name=k, offset=i*4, datatype=self.Field.FLOAT32, count=1)
                             for i, k in enumerate(('x', 'y', 'z'))]
            output.is_bigendian, output.is_dense = False, True
            output.point_step, output.row_step = 12, 12*len(points)
            output.data = points.tobytes()
            if not fresh(stamp, time.monotonic()):
                continue
            self.publisher.publish(output)  # Empty fresh clouds are valid.
            self.heartbeat.publish(output.header)
            self.last_published = stamp
            stats.update(source_age_s=round(time.monotonic()-stamp, 4), compensated=self.predictions)
            reason = 'projected ' + json.dumps(stats)
            if not self.announced:
                self.node.get_logger().info('[VOA_COSTMAP] first fresh projection published')
                self.announced = True
            break
        if now-self.last_log >= 5:
            self.node.get_logger().info('[VOA_COSTMAP] ' + reason)
            self.last_log = now


def main():
    import argparse
    import yaml
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from grid_map_msgs.msg import GridMap
    from tf2_ros import Buffer, TransformListener
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    # One obstacle threshold for VOA braking and Nav2 planning, no duplicate
    # parameter to silently drift away from the factory predicate.
    params = yaml.safe_load(args.config.read_text())['/**']['ros__parameters']
    minimum = float(params['kObstatleHeight'])
    if not math.isfinite(minimum) or minimum <= 0:
        raise ValueError('invalid VOA obstacle height')
    rclpy.init()
    node = rclpy.create_node('patrol_voa_costmap_bridge')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    bridge = Bridge(node, buffer, PointCloud2, PointField, Header, Time, minimum)
    def on_grid(message):
        bridge.receive(message)
        bridge.publish()
    def on_registration(message):
        bridge.registration(message)
        bridge.publish()
    latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscription = node.create_subscription(GridMap, '/patrol/voa/grid_map', on_grid, latest_qos)
    registration = node.create_subscription(Odometry, '/patrol/registration_odom', on_registration, 1)
    timer = node.create_timer(.025, bridge.publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
