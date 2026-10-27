#!/usr/bin/env python3
"""Carry RViz topics and isolated goal requests over one authenticated SSH pipe.

The Jetson keeps all navigation and motion supervision. No velocity, arming,
initial-pose, or action messages are accepted from this display transport.
"""
import argparse
from collections import deque
import os
import shlex
import struct
import subprocess
import sys
import threading
import time


HEADER = struct.Struct('!4sHI')
MAGIC = b'L3RV'
MAX_PAYLOAD = 8 * 1024 * 1024
# topic, ROS type, max Hz (0 = every update), latched, ordered events
TOPICS = (
    ('/map', 'nav_msgs/msg/OccupancyGrid', 0, True, True),
    ('/tf', 'tf2_msgs/msg/TFMessage', 20, False, False),
    ('/tf_static', 'tf2_msgs/msg/TFMessage', 0, True, False),
    ('/aligned_points', 'sensor_msgs/msg/PointCloud2', 2, False, False),
    ('/global_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', 0, True, True),
    ('/global_costmap/costmap_updates', 'map_msgs/msg/OccupancyGridUpdate', 0, False, True),
    ('/local_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', 0, True, True),
    ('/local_costmap/costmap_updates', 'map_msgs/msg/OccupancyGridUpdate', 0, False, True),
    ('/plan', 'nav_msgs/msg/Path', 0, True, True),
    ('/local_plan', 'nav_msgs/msg/Path', 5, False, False),
    ('/global_costmap/published_footprint', 'geometry_msgs/msg/PolygonStamped', 2, False, False),
    ('/local_costmap/published_footprint', 'geometry_msgs/msg/PolygonStamped', 2, False, False),
    ('/odom', 'nav_msgs/msg/Odometry', 10, False, False),
    ('/patrol/goal_bridge_state', 'std_msgs/msg/String', 2, True, False),
    ('/patrol/localization_ready', 'std_msgs/msg/Bool', 2, True, False),
    ('/patrol/gate_state', 'std_msgs/msg/String', 2, False, False),
    ('/patrol/armed', 'std_msgs/msg/Bool', 2, False, False),
)
GOAL_ID = 100
GOAL_TOPIC = '/patrol/goal_request'


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        chunk = stream.read(count - len(data))
        if not chunk:
            raise EOFError('SSH stream closed')
        data.extend(chunk)
    return bytes(data)


def read_packet(stream):
    magic, ident, size = HEADER.unpack(read_exact(stream, HEADER.size))
    if magic != MAGIC or size > MAX_PAYLOAD:
        raise ValueError('invalid RViz transport frame')
    return ident, read_exact(stream, size)


def write_packet(stream, ident, payload):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('RViz message exceeds transport limit')
    stream.write(HEADER.pack(MAGIC, ident, len(payload)) + payload)
    stream.flush()


def validate_direction(side, ident):
    allowed = ident == GOAL_ID if side == 'remote' else 0 <= ident < len(TOPICS)
    if not allowed:
        raise ValueError('message is not allowed in this transport direction')


class OutboundBuffer:
    """Coalesce display samples, preserving full-map/update and goal ordering."""
    def __init__(self):
        self.condition = threading.Condition()
        self.queue = deque()
        self.latest = {}
        self.events = 0

    def put(self, ident, payload, ordered=False):
        with self.condition:
            if ordered:
                if self.events >= 128:
                    raise OverflowError('RViz event transport is backlogged')
                self.queue.append((ident, payload))
                self.events += 1
            else:
                if ident not in self.latest:
                    self.queue.append((ident, None))
                self.latest[ident] = payload
            self.condition.notify()

    def get(self):
        with self.condition:
            if not self.queue:
                self.condition.wait(.2)
            if not self.queue:
                return None
            ident, payload = self.queue.popleft()
            if payload is None:
                payload = self.latest.pop(ident)
            else:
                self.events -= 1
            return ident, payload


def run(args):
    # Import ROS only in the endpoints; protocol tests need no ROS environment.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import serialize_message, deserialize_message
    from rosidl_runtime_py.utilities import get_message

    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'
    remote = args.side == 'remote'
    child = None
    if remote:
        incoming, outgoing = sys.stdin.buffer, sys.stdout.buffer
    else:
        command = 'source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh >/dev/null 2>&1; '
        command += 'exec env RCUTILS_LOGGING_USE_STDOUT=0 PYTHONUNBUFFERED=1 python3 '
        command += shlex.quote(args.remote_script) + ' --side remote'
        child = subprocess.Popen(
            ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
             '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
             '-i', args.key, args.robot, command],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        incoming, outgoing = child.stdout, child.stdin

    rclpy.init(args=[])
    node = Node('patrol_rviz_ssh_' + args.side)
    stop = threading.Event()
    errors = []
    outbound = OutboundBuffer()
    received = OutboundBuffer()
    last_sent = {}
    transforms = {1: {}, 2: {}}
    publishers = {}
    types = {}
    first = set()

    def qos(latched=False, sensor=False, events=False):
        result = QoSProfile(depth=20 if events else 1)
        result.durability = DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE
        result.reliability = ReliabilityPolicy.BEST_EFFORT if sensor else ReliabilityPolicy.RELIABLE
        return result

    def fail(error):
        errors.append(str(error))
        stop.set()

    def callback(ident):
        def send(message):
            try:
                if ident in transforms:
                    for transform in message.transforms:
                        transforms[ident][transform.child_frame_id] = transform
                    message.transforms = list(transforms[ident].values())
                rate = TOPICS[ident][2] if ident != GOAL_ID else 0
                now = time.monotonic()
                if rate and now - last_sent.get(ident, 0) < 1 / rate:
                    return
                last_sent[ident] = now
                ordered = ident == GOAL_ID or TOPICS[ident][4]
                outbound.put(ident, serialize_message(message), ordered)
                if ident == GOAL_ID:
                    p = message.pose.position
                    node.get_logger().info('RViz goal sent over SSH: x={:.3f}, y={:.3f}'.format(p.x, p.y))
            except Exception as error:
                fail(error)
        return send

    for ident, (topic, type_name, _rate, latched, ordered) in enumerate(TOPICS):
        message_type = get_message(type_name)
        types[ident] = message_type
        if remote:
            node.create_subscription(message_type, topic, callback(ident),
                                     qos(latched, ident in (1, 3), ordered))
        else:
            publishers[ident] = node.create_publisher(message_type, topic, qos(latched, events=ordered))
    goal_type = get_message('geometry_msgs/msg/PoseStamped')
    types[GOAL_ID] = goal_type
    if remote:
        publishers[GOAL_ID] = node.create_publisher(goal_type, GOAL_TOPIC, 10)
    else:
        node.create_subscription(goal_type, GOAL_TOPIC, callback(GOAL_ID), 10)

    def reader():
        try:
            while not stop.is_set():
                ident, payload = read_packet(incoming)
                validate_direction(args.side, ident)
                received.put(ident, payload, ident == GOAL_ID or TOPICS[ident][4])
        except Exception as error:
            fail(error)

    def writer():
        try:
            while not stop.is_set():
                packet = outbound.get()
                if packet is not None:
                    write_packet(outgoing, *packet)
        except Exception as error:
            fail(error)

    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=writer, daemon=True).start()

    def publish_received():
        # Drain a bounded batch on the ROS thread, without blocking callbacks.
        for _ in range(64):
            with received.condition:
                if not received.queue:
                    break
            ident, payload = received.get()
            message = deserialize_message(payload, types[ident])
            publishers[ident].publish(message)
            if ident not in first:
                first.add(ident)
                name = GOAL_TOPIC if ident == GOAL_ID else TOPICS[ident][0]
                node.get_logger().info('first SSH message delivered: ' + name)

    node.create_timer(.02, publish_received)
    node.get_logger().info('RViz SSH transport started: ' + args.side)
    try:
        while rclpy.ok() and not stop.is_set():
            if args.parent_pid:
                try:
                    os.kill(args.parent_pid, 0)
                except ProcessLookupError:
                    break
            rclpy.spin_once(node, timeout_sec=.2)
        if errors:
            node.get_logger().error('RViz SSH transport closed: ' + errors[0])
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if child is not None:
            child.stdin.close()
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                child.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if errors and not remote else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('local', 'remote'), default='local')
    parser.add_argument('--robot', default='ysc@10.119.1.171')
    parser.add_argument('--key', default='/home/duyipei/.ssh/id_ed25519_cyberdog')
    parser.add_argument('--remote-script', default='/home/ysc/lite3_patrol_project/scripts/rviz_ssh_bridge.py')
    parser.add_argument('--parent-pid', type=int, default=0)
    raise SystemExit(run(parser.parse_args()))


if __name__ == '__main__':
    main()
