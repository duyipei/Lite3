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
import threading
import time
from types import SimpleNamespace

import numpy as np

TOPIC = '/patrol/voa/costmap_obstacles'
HEARTBEAT = '/patrol/voa/costmap_heartbeat'
MARKING_FRAME = 'patrol_voa_costmap_origin'
MAX_AGE = .5
MAX_PREDICTION = .5
# Keep an explicit margin below the velocity gate's 0.5 s freshness so a
# projection that is already old when it reaches the gate cannot be rejected.
PUBLISH_MAX_AGE = .35
# Bound the work done by one timer tick. The newest few frames are the only
# ones that can still satisfy PUBLISH_MAX_AGE after TF lookup.
MAX_PENDING_TRIES = 4


def stamp_seconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('invalid sensor stamp')
    return stamp.sec + stamp.nanosec / 1e9


def fresh(stamp, now, max_age=MAX_AGE):
    # Small controller/host timestamp skew is already present on this Lite3.
    return math.isfinite(stamp) and stamp > 0 and -.05 <= now - stamp <= max_age


# The same decoder is used inside the velocity gate, before any map TF wait.
try:
    from lite3_patrol_nav.voa_grid import grid_obstacles
except ModuleNotFoundError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'patrol_nav_ws/src/lite3_patrol_nav'))
    from lite3_patrol_nav.voa_grid import grid_obstacles


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


def marking_origin(transform):
    """2-D marking frame, NOT a physical sensor/extrinsic/registration TF.

    Cloud Z is synthetic .5, so range must also originate in that same plane.
    Use the very pose that projected XY, never a later/older localization pose.
    """
    matrix=transform if isinstance(transform,np.ndarray) else transform_matrix(transform)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all():
        raise ValueError('invalid marking origin')
    yaw=math.atan2(matrix[1,0],matrix[0,0])
    return (float(matrix[0,3]),float(matrix[1,3]),.5), (0.,0.,math.sin(yaw/2),math.cos(yaw/2))


class Bridge:
    def __init__(self, node, buffer, cloud_type, field_type, header_type, time_type, minimum,
                 output_topic=TOPIC, heartbeat_topic=HEARTBEAT, include_unknown=True, origin_publisher=None):
        self.node, self.buffer = node, buffer
        self.Cloud, self.Field, self.Header, self.Time = cloud_type, field_type, header_type, time_type
        self.minimum = minimum
        self.include_unknown = include_unknown
        self.origin_publisher = origin_publisher
        self.pending = deque(maxlen=16)
        self.last_received = self.last_published = 0.
        self.last_publish_time = 0.
        self.last_log = 0.
        self.last_gap_log = 0.
        self.last_timer = 0.
        self.fail_project = 0
        self.fail_grid = 0
        self.last_failure = 'none'
        self.announced = False
        self.anchors = deque(maxlen=12)
        self.predictions = 0
        # MultiThreadedExecutor runs the TF listener, grid callback and timer
        # concurrently; this lock keeps Bridge state consistent.
        self.lock = threading.RLock()
        self.publish_lock = threading.Lock()
        self.publisher = node.create_publisher(cloud_type, output_topic, 1)
        self.heartbeat = node.create_publisher(header_type, heartbeat_topic, 1)

    def receive(self, msg):
        with self.lock:
            stamp = stamp_seconds(msg.header.stamp)
            if fresh(stamp, time.monotonic()) and stamp > self.last_received:
                self.pending.append(msg)
                self.last_received = stamp

    def registration(self, msg):
        with self.lock:
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
        with self.lock:
            anchors = tuple(reversed(self.anchors))
        if not anchors:
            raise ValueError('no recent NDT anchor')
        # This endpoint is common to every anchor. If the grid-time leg TF is
        # pending, trying it twelve times cannot help; try an older grid next.
        leg1 = self.buffer.lookup_transform('robot_odom', 'leg_base_link', target_time)
        for anchor in anchors:
            age = stamp-stamp_seconds(anchor.header.stamp)
            if not 0 <= age <= MAX_PREDICTION:
                continue
            try:
                leg0 = self.buffer.lookup_transform('robot_odom', 'leg_base_link', self.Time.from_msg(anchor.header.stamp))
            except Exception:
                continue  # An older bounded anchor can still have both TFs.
            pose = anchor.pose.pose
            result = compensated_pose(SimpleNamespace(translation=pose.position, rotation=pose.orientation),
                                      leg0.transform, leg1.transform)
            self.predictions += 1
            return result
        raise ValueError('no recent NDT anchor with timestamped leg odometry')

    def publish(self):
        # A delayed timer must not occupy the executor's other worker waiting
        # for a big lock. Leave it free to receive TF, registration and grids.
        if not self.publish_lock.acquire(blocking=False):
            return
        try:
            self._publish()
        finally:
            self.publish_lock.release()

    def _publish(self):
        now = time.monotonic()
        if self.last_timer and now - self.last_timer > .2:
            self.node.get_logger().info(
                '[VOA_COSTMAP] timer_delayed {:.3f}s'.format(now - self.last_timer))
        self.last_timer = now
        # NDT runs at 10 Hz. Keep a bounded backlog so a newer camera frame
        # does not continually replace the one waiting for interpolated TF.
        reason = 'waiting for fresh grid and matching NDT TF'
        with self.lock:
            candidates = list(reversed(self.pending))[:MAX_PENDING_TRIES]
        for msg in candidates:
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= self.last_published or not fresh(stamp, now, PUBLISH_MAX_AGE):
                continue
            try:
                transform = self.projection_transform(msg)
            except Exception as error:
                self.fail_project += 1
                self.last_failure = '{}:{}'.format(type(error).__name__, str(error)[:80])
                continue  # Never fall back to latest pose for an older frame.
            try:
                xyz, stats = grid_obstacles(msg, self.minimum, include_unknown=self.include_unknown)
                points = project_to_map(xyz, transform)
            except (ValueError, TypeError, AttributeError, IndexError) as error:
                self.fail_grid += 1
                self.last_failure = 'grid:{}:{}'.format(type(error).__name__, str(error)[:80])
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
            published_at = time.monotonic()
            if not fresh(stamp, published_at, PUBLISH_MAX_AGE):
                continue
            if self.origin_publisher is not None:
                try:
                    self.origin_publisher(msg.header.stamp,transform)
                except Exception as error:
                    self.fail_project += 1
                    self.last_failure = 'marking_origin:'+str(error)[:80]
                    continue  # No cloud/health when its origin was not sent.
            published_at = time.monotonic()
            if not fresh(stamp,published_at,PUBLISH_MAX_AGE):
                continue
            self.publisher.publish(output)  # Empty fresh clouds are valid.
            self.heartbeat.publish(output.header)
            self.last_published = stamp
            self.last_publish_time = published_at
            stats.update(source_age_s=round(published_at-stamp, 4), compensated=self.predictions)
            reason = 'projected ' + json.dumps(stats)
            if not self.announced:
                self.node.get_logger().info('[VOA_COSTMAP] first fresh projection published')
                self.announced = True
            break
        if (self.last_publish_time
                and now - self.last_publish_time > PUBLISH_MAX_AGE
                and now - self.last_gap_log >= 1.0):
            self.node.get_logger().info(
                '[VOA_COSTMAP] heartbeat_gap age={:.3f} pending={} anchors={} '
                'fail_project={} fail_grid={} last={}'.format(
                    now - self.last_publish_time, len(self.pending),
                    len(self.anchors), self.fail_project, self.fail_grid,
                    self.last_failure))
            self.last_gap_log = now
        if now-self.last_log >= 5:
            self.node.get_logger().info('[VOA_COSTMAP] ' + reason)
            self.last_log = now


def main():
    import argparse
    import yaml
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from grid_map_msgs.msg import GridMap
    from tf2_ros import Buffer, TransformListener
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--known-obstacles-only', action='store_true',
                        help='Factory-only: unknown is vetoed by direct grid stop, not marked as a physical wall')
    parser.add_argument('--publish-costmap-origin', action='store_true')
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
    publish_origin = None
    if args.publish_costmap_origin:
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster
        origin_broadcaster=TransformBroadcaster(node)
        def publish_origin(stamp,pose):
            xyz,quat=marking_origin(pose)
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id=MARKING_FRAME
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=xyz
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=quat
            origin_broadcaster.sendTransform(t)
    bridge = Bridge(node, buffer, PointCloud2, PointField, Header, Time, minimum,
                    include_unknown=not args.known_obstacles_only,origin_publisher=publish_origin)
    def on_grid(message):
        # Only enqueue here. Projection is done by the 25 ms timer so the
        # 30 Hz grid callback cannot starve the TF listener callbacks that the
        # projection itself needs. Under Nav2 load the old callback path could
        # leave >0.5 s heartbeat gaps and trip the velocity gate.
        bridge.receive(message)
    def on_registration(message):
        bridge.registration(message)
    latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    group = ReentrantCallbackGroup()
    subscription = node.create_subscription(
        GridMap, '/patrol/voa/grid_map', on_grid, latest_qos,
        callback_group=group)
    registration = node.create_subscription(
        Odometry, '/patrol/registration_odom', on_registration, 1,
        callback_group=group)
    timer = node.create_timer(.025, bridge.publish, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
