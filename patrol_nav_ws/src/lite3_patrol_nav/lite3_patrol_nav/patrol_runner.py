#!/usr/bin/env python3
"""Offline validation or explicitly authorized leased goal-bridge patrol.

No direct Nav2 action client. ROS is imported only for --execute.
"""
import argparse
import hashlib
import json
import math
import re
import signal
import time
from pathlib import Path

import yaml


def load_mission(path):
    mission = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(mission, dict) or mission.get('frame_id') != 'map':
        raise ValueError('mission frame_id must be map')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', str(mission.get('map_id', ''))):
        raise ValueError('mission must specify map_id')
    points = mission.get('waypoints')
    if not isinstance(points, list) or not 1 <= len(points) <= 500:
        raise ValueError('mission must contain 1..500 waypoints')
    ids = set()
    for point in points:
        if not isinstance(point, dict) or not isinstance(point.get('id'), str) or not point['id'] or point['id'] in ids:
            raise ValueError('waypoint IDs must be nonempty unique strings')
        ids.add(point['id'])
        for key in ('x', 'y', 'yaw', 'pause_s'):
            value = point.get(key, 0.0 if key == 'pause_s' else None)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError('invalid {} in {}'.format(key, point['id']))
        if not 0 <= point.get('pause_s', 0) <= 600:
            raise ValueError('pause_s must be between 0 and 600')
    return mission


def verify_map(mission, map_yaml):
    path = Path(map_yaml).resolve()
    if path.parent.name != mission['map_id']:
        raise ValueError('mission/map directory mismatch')
    config = yaml.safe_load(path.read_text())
    image = path.parent / config['image']
    for key, file in [('map_yaml_sha256', path), ('map_image_sha256', image)]:
        expected = mission.get(key)
        if not isinstance(expected, str) or hashlib.sha256(file.read_bytes()).hexdigest() != expected:
            raise ValueError('map fingerprint missing or changed: ' + key)


def classify_event(data, token, index):
    if data.get('token') != token or data.get('index') != index:
        return 'ignore'
    state = data.get('state', '')
    if state.startswith('RESULT:'):
        return 'success' if re.fullmatch(r'RESULT:GOAL_\d+:SUCCEEDED(?::READY_FOR_NEXT_GOAL)?', state) else 'failure'
    if state.startswith(('REJECTED:', 'CANCEL', 'RESULT_ERROR:', 'MISSION_STOPPED:')):
        return 'failure'
    return 'pending'


def execute(mission, goal_timeout):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    class Runner(Node):
        def __init__(self):
            super().__init__('patrol_runner')
            self.token = None
            self.index = 0
            self.event = ''
            self.result = None
            self.failure = None
            self.ready = False
            self.armed = None
            self.ready_time = self.armed_time = 0.0
            self.arm_seen = False
            self.closing = False
            self.pose_sample = None
            self.velocity_sample = None
            self.target = None
            self.heartbeat_pub = self.create_publisher(String, '/patrol/mission_heartbeat', 10)
            self.goal_pub = self.create_publisher(String, '/patrol/mission_goal', 10)
            self.create_subscription(Bool, '/patrol/localization_ready', self.on_ready, 10)
            self.create_subscription(Bool, '/patrol/armed', self.on_armed, 10)
            self.create_subscription(String, '/patrol/mission_event', self.on_event, 10)
            self.acquire = self.create_client(Trigger, '/patrol/acquire_mission')
            self.cancel = self.create_client(Trigger, '/patrol/cancel_goal')
            self.arm = self.create_client(SetBool, '/patrol/set_armed')
            self.create_timer(.2, self.heartbeat)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_subscription(Twist, '/cmd_vel_corrected', self.on_velocity, 10)
            self.create_timer(1.0, self.report_motion)

        def on_odom(self, msg):
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.pose_sample = (p.position.x, p.position.y, yaw, msg.twist.twist.angular.z)

        def on_velocity(self, msg):
            self.velocity_sample = (msg.linear.x, msg.angular.z)

        def report_motion(self):
            if not self.token or self.closing or self.pose_sample is None or self.target is None:
                return
            x, y, yaw, measured_w = self.pose_sample
            bearing = math.atan2(self.target['y']-y, self.target['x']-x)
            error = math.atan2(math.sin(bearing-yaw), math.cos(bearing-yaw))
            print('[MOTION] wp={} xy=({:.3f},{:.3f}) yaw_deg={:.1f} bearing_error_deg={:.1f} target_yaw_deg={:.1f} odom_w={:.3f} cmd_v_w={}'.format(
                self.index,x,y,math.degrees(yaw),math.degrees(error),math.degrees(self.target['yaw']),measured_w,self.velocity_sample),flush=True)

        def on_ready(self, msg):
            self.ready, self.ready_time = msg.data, time.monotonic()
            if self.token and not msg.data:
                self.failure = 'localization revoked'

        def on_armed(self, msg):
            self.armed, self.armed_time = msg.data, time.monotonic()
            if self.token and msg.data:
                self.arm_seen = True
            if self.token and self.arm_seen and not msg.data and not self.closing:
                self.failure = 'gate disarmed; automatic re-arm is forbidden'

        def on_event(self, msg):
            try:
                data = json.loads(msg.data)
                result = classify_event(data, self.token, self.index)
            except (ValueError, TypeError, AttributeError):
                return
            if result == 'ignore':
                return
            self.event = data['state']
            if result == 'failure':
                self.failure = self.event
            elif result == 'success':
                self.result = True

        def heartbeat(self):
            if self.token and not self.closing:
                message = String(); message.data = self.token
                self.heartbeat_pub.publish(message)

        def check(self):
            if self.failure:
                raise RuntimeError(self.failure)
            if self.token and (not self.ready or time.monotonic()-self.ready_time > 1.5
                               or time.monotonic()-self.armed_time > 1.5):
                raise RuntimeError('localization/gate heartbeat stale')

        def wait(self, condition, timeout):
            deadline = time.monotonic()+timeout
            while True:
                rclpy.spin_once(self, timeout_sec=.05)
                self.check()
                if condition():
                    return
                if time.monotonic() >= deadline:
                    raise RuntimeError('timeout: ' + (self.event or 'waiting for bridge'))

        def delay(self, seconds):
            deadline = time.monotonic()+seconds
            self.wait(lambda: time.monotonic() >= deadline, seconds+1)

        def call(self, client, request, supervise=True):
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('service unavailable: ' + client.srv_name)
            future = client.call_async(request)
            deadline = time.monotonic()+4
            while not future.done():
                rclpy.spin_once(self, timeout_sec=.05)
                if supervise:
                    self.check()
                if time.monotonic() >= deadline:
                    raise RuntimeError('service response timeout: ' + client.srv_name)
            response = future.result()
            if not response or not response.success:
                raise RuntimeError('service rejected: ' + str(getattr(response, 'message', None)))
            return response

        def run(self):
            self.wait(lambda: self.ready and self.armed is False and
                      time.monotonic()-self.ready_time < 1.5 and
                      time.monotonic()-self.armed_time < 1.5, 10)
            lease = json.loads(self.call(self.acquire, Trigger.Request()).message)
            self.token = lease['token']
            if lease['map_id'] != mission['map_id']:
                raise RuntimeError('running navigation map differs from mission')
            self.heartbeat()
            self.wait(lambda: self.goal_pub.get_subscription_count() > 0, 5)
            for index, point in enumerate(mission['waypoints'], 1):
                self.check()
                self.index, self.event, self.result = index, '', None
                self.target = point
                message = String()
                message.data = json.dumps(dict(token=self.token, index=index,
                                              x=point['x'], y=point['y'], yaw=point['yaw']))
                print('[GOAL] {}/{} {}'.format(index, len(mission['waypoints']), point['id']), flush=True)
                self.goal_pub.publish(message)
                if index == 1:
                    self.wait(lambda: self.event.startswith('QUEUED:'), 10)
                    self.delay(.75)
                    request = SetBool.Request(); request.data = True
                    self.call(self.arm, request)
                    self.wait(lambda: self.armed is True, 3)
                self.wait(lambda: self.result is True, goal_timeout)
                print('[DONE] ' + point['id'], flush=True)
                self.delay(max(.75, point.get('pause_s', 0)))

        def stop(self):
            if not self.token:
                return
            self.closing = True
            errors = []
            request = SetBool.Request(); request.data = False
            for client, req in [(self.arm, request), (self.cancel, Trigger.Request())]:
                try:
                    self.call(client, req, supervise=False)
                except Exception as error:
                    errors.append(str(error))
            self.token = None
            if errors:
                raise RuntimeError('STOP not fully acknowledged; use remote emergency stop: ' + '; '.join(errors))
            print('[STOP] DISARM acknowledged; goal cancellation requested', flush=True)

    def interrupted(_sig, _frame):
        raise KeyboardInterrupt()

    previous = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    rclpy.init(args=[])
    for sig in previous:
        signal.signal(sig, interrupted)
    node = Runner()
    try:
        node.run()
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    print('[MISSION SUCCEEDED] all waypoints completed; DISARMED', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission', type=Path)
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--authorize-draft', action='store_true', help='One-run operator approval; does not modify the draft')
    parser.add_argument('--goal-timeout', type=float, default=120.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0:
        parser.error('goal-timeout must be finite and positive')
    mission = load_mission(args.mission)
    verify_map(mission, args.map_yaml or args.mission.parent/'patrol_map.yaml')
    print('mission: {} waypoints, map={}'.format(len(mission['waypoints']), mission['map_id']), flush=True)
    if not args.execute:
        print('DRY RUN ONLY: schema and map fingerprints checked; NO ROS connection or path certification')
        return
    if not args.authorize_draft and (mission.get('enabled') is not True or mission.get('review_required') is not False):
        parser.error('disabled draft: use interactive patrol_route.sh run to authorize this one run')
    execute(mission, args.goal_timeout)


if __name__ == '__main__':
    main()
