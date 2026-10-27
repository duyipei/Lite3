#!/usr/bin/env python3
"""Supply the camera mount from this Lite3's factory voa_launch.py.

Reuse a matching existing TF; refuse a conflicting mount or disconnected parent.
This node publishes only /tf_static, never velocity commands.
"""
import math
import time

XYZ = (0.25489, 0.0, 0.07249)
QUAT = (0.0, math.sin(0.34907 / 2), 0.0, math.cos(0.34907 / 2))


def matches_mount(xyz, quat):
    if not all(math.isfinite(v) for v in (*xyz, *quat)):
        return False
    position_error = math.sqrt(sum((a-b)**2 for a, b in zip(xyz, XYZ)))
    norm = math.sqrt(sum(v*v for v in quat))
    if abs(norm-1.0) > 0.01:
        return False
    dot = abs(sum(a*b/norm for a, b in zip(quat, QUAT)))
    angle = 2 * math.acos(min(1.0, dot))
    return position_error <= 0.002 and angle <= 0.005


def main():
    import rclpy
    import yaml
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
    rclpy.init()
    node = rclpy.create_node('patrol_voa_camera_mount')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        existing = buffer.lookup_transform('base_link', 'camera_link', Time())
    except Exception:
        existing = None
    if existing is not None:
        t, q = existing.transform.translation, existing.transform.rotation
        if not matches_mount((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)):
            raise RuntimeError('existing camera mount disagrees with factory calibration')
        node.get_logger().info('Reusing verified base_link -> camera_link transform')
    else:
        frames = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
        if 'camera_link' in frames:
            raise RuntimeError('camera_link already has a disconnected parent; refusing to reparent')
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = XYZ
        q.x, q.y, q.z, q.w = QUAT
        broadcaster.sendTransform(transform)
        node.get_logger().info('Publishing factory base_link -> camera_link transform')
    # Calibration is checked once above. Keep the static broadcaster alive,
    # without continuing to deserialize every high-rate odometry TF message.
    listener.unregister()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
