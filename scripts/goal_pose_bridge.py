#!/usr/bin/env python3
"""Bridge an isolated PoseStamped request topic to the Jetson Foxy Nav2 action."""

import math
import time
import json
import uuid
from pathlib import Path as FilePath

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import PointCloud2
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__("patrol_goal_pose_bridge")
        self.localization_ready = False
        self.goal_handle = None
        self.goal_active = False
        self.goal_pending = False
        self.pending_goal = None
        self.pending_serial = None
        self.armed = False
        self.request_serial = 0
        self.current_state = "STARTING"
        self.action_server_ready = False
        self.last_goal_pose = None
        self.last_goal_time = 0.0
        self.duplicate_goal_window_s = 1.0
        self.declare_parameter('active_map_id', '')
        self.declare_parameter('goal_clearance_m', 0.45)
        self.declare_parameter('require_voa_obstacles', False)
        self.goal_clearance_m = float(self.get_parameter('goal_clearance_m').value)
        self.declare_parameter('goal_nav2_file', '')
        self.declare_parameter('goal_response_file', '')
        self.goal_geometry = None
        nav_file = self.get_parameter('goal_nav2_file').value
        if nav_file:
            import yaml
            try:
                from lite3_patrol_nav.obstacle_safety import footprint_extents, terminal_extents
            except ModuleNotFoundError:
                import sys
                sys.path.insert(0, str(FilePath(__file__).resolve().parents[1]/'patrol_nav_ws/src/lite3_patrol_nav'))
                from lite3_patrol_nav.obstacle_safety import footprint_extents, terminal_extents
            nav = yaml.safe_load(FilePath(nav_file).read_text())
            extents = np.maximum(*[footprint_extents(nav[k][k]['ros__parameters'])
                                   for k in ('local_costmap', 'global_costmap')])
            tolerance = nav['controller_server']['ros__parameters']['goal_checker']
            profile = yaml.safe_load(FilePath(self.get_parameter('goal_response_file').value).read_text())
            self.goal_geometry = terminal_extents(*extents, tolerance['yaw_goal_tolerance'],
                                                 tolerance['xy_goal_tolerance'], profile['goal_extra_margin_m'])
        self.require_voa_obstacles = bool(
            self.get_parameter('require_voa_obstacles').value)
        self.map_msg = None
        self.voa_cloud = None
        self.voa_cloud_time = 0.0
        self.mission_token = None
        self.mission_index = 0
        self.mission_last_heartbeat = 0.0
        self.mission_dispatch = False
        self.ready_received = 0.0
        self.armed_received = 0.0
        self.mission_had_arm = False
        self.disarm_future = None
        self.disarm_required = False
        self.disarm_sent = 0.0

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_publisher = self.create_publisher(
            String, "/patrol/goal_bridge_state", state_qos
        )
        self.bridge_ready_publisher = self.create_publisher(
            Bool, "/patrol/goal_bridge_ready", state_qos
        )
        self.goal_active_publisher = self.create_publisher(
            Bool, "/patrol/goal_active", state_qos
        )
        self.goal_pending_publisher = self.create_publisher(
            Bool, "/patrol/goal_pending", state_qos
        )
        self.preview_path_publisher = self.create_publisher(
            Path, "/plan", state_qos
        )
        self.mission_event_publisher = self.create_publisher(String, '/patrol/mission_event', 10)
        self.disarm_client = self.create_client(SetBool, '/patrol/set_armed')
        self.create_service(Trigger, '/patrol/acquire_mission', self.acquire_mission)
        self.create_subscription(String, '/patrol/mission_heartbeat', self.mission_heartbeat, 10)
        self.create_subscription(String, '/patrol/mission_goal', self.mission_goal, 10)

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            "/patrol/localization_ready",
            self.ready_callback,
            ready_qos,
        )
        self.create_subscription(
            PoseStamped, "/patrol/goal_request", self.goal_callback, 10
        )
        self.create_subscription(Bool, "/patrol/armed", self.armed_callback, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos)
        voa_qos = QoSProfile(depth=1)
        voa_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            PointCloud2, "/patrol/voa/costmap_obstacles",
            self.voa_cloud_callback, voa_qos)
        self.create_service(Trigger, "/patrol/cancel_goal", self.cancel_callback)
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.planner_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.create_timer(0.5, self.publish_health)
        self.publish_state("WAITING:NAV2_ACTION_SERVER")
        self.publish_bridge_ready(False)
        self.publish_goal_active(False)
        self.publish_goal_pending(False)
        self.get_logger().info(
            "bridging isolated /patrol/goal_request to Foxy /navigate_to_pose; "
            "the first goal requires ARM and later goals run in the same "
            "explicitly armed session"
        )

    def publish_state(self, text):
        self.current_state = text
        message = String()
        message.data = text
        self.state_publisher.publish(message)
        if self.mission_token:
            event = String()
            event.data = json.dumps(dict(token=self.mission_token, index=self.mission_index, state=text))
            self.mission_event_publisher.publish(event)

    def acquire_mission(self, _request, response):
        now = time.monotonic()
        map_id = self.get_parameter('active_map_id').value
        if (self.mission_token or self.goal_active or self.goal_pending or self.goal_handle
                or self.armed or self.disarm_required or now-self.armed_received > 1.5
                or not self.localization_ready or now-self.ready_received > 1.5
                or not self.action_client.server_is_ready() or not map_id):
            response.success = False
            response.message = 'requires fresh READY, DISARMED, idle bridge and configured map'
            return response
        self.mission_token = uuid.uuid4().hex
        self.mission_index = 0
        self.mission_had_arm = False
        self.mission_last_heartbeat = now
        response.success = True
        response.message = json.dumps(dict(token=self.mission_token, map_id=map_id))
        return response

    def mission_heartbeat(self, message):
        if self.mission_token and message.data == self.mission_token:
            self.mission_last_heartbeat = time.monotonic()

    def mission_goal(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
            if not self.mission_token or data.get('token') != self.mission_token:
                return
            if (type(data.get('index')) is not int or data['index'] != self.mission_index+1
                    or self.goal_active or self.goal_pending):
                raise ValueError('out-of-order or concurrent mission goal')
            values = [data[k] for k in ('x', 'y', 'yaw')]
            if any(type(v) not in (float, int) or not math.isfinite(v) for v in values):
                raise ValueError('invalid mission coordinates')
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = map(float, values[:2])
            pose.pose.orientation.z = math.sin(values[2]/2)
            pose.pose.orientation.w = math.cos(values[2]/2)
            self.mission_index = data['index']
            self.mission_dispatch = True
            self.goal_callback(pose)
        except (ValueError, KeyError, TypeError) as error:
            if self.mission_token:
                self.stop_mission('invalid_request:' + str(error))
        finally:
            self.mission_dispatch = False

    def stop_mission(self, reason):
        self.publish_state('MISSION_STOPPED:' + reason)
        self.mission_token = None
        self.request_serial += 1  # Reject/cancel late action acceptances/results.
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.pending_goal = None
        self.pending_serial = None
        self.publish_goal_pending(False)
        self.publish_goal_active(False)
        self.disarm_required = True
        self.request_disarm()

    def request_disarm(self):
        if not self.disarm_required or not self.disarm_client.service_is_ready():
            return
        if self.disarm_future is not None:
            if self.disarm_future.done():
                try:
                    if self.disarm_future.result().success:
                        self.disarm_required = False
                        self.disarm_future = None
                        return
                except Exception:
                    pass
            elif time.monotonic()-self.disarm_sent < 2.0:
                return
        request = SetBool.Request()
        request.data = False
        self.disarm_future = self.disarm_client.call_async(request)
        self.disarm_sent = time.monotonic()

    def publish_bridge_ready(self, ready):
        message = Bool()
        message.data = bool(ready)
        self.bridge_ready_publisher.publish(message)

    def publish_goal_active(self, active):
        self.goal_active = bool(active)
        message = Bool()
        message.data = self.goal_active
        self.goal_active_publisher.publish(message)

    def publish_goal_pending(self, pending):
        self.goal_pending = bool(pending)
        message = Bool()
        message.data = self.goal_pending
        self.goal_pending_publisher.publish(message)

    def publish_health(self):
        if self.mission_token:
            now = time.monotonic()
            if now-self.mission_last_heartbeat > 2.0:
                self.stop_mission('runner_heartbeat_timeout')
            elif not self.localization_ready or now-self.ready_received > 1.5:
                self.stop_mission('localization_not_ready_or_stale')
            elif now-self.armed_received > 1.5:
                self.stop_mission('gate_heartbeat_stale')
            elif not self.action_client.server_is_ready():
                self.stop_mission('navigation_server_lost')
        self.request_disarm()
        server_ready = self.action_client.server_is_ready()
        if server_ready != self.action_server_ready:
            self.action_server_ready = server_ready
            if server_ready:
                self.get_logger().info("Foxy /navigate_to_pose action server is ready")
                if self.current_state == "WAITING:NAV2_ACTION_SERVER":
                    self.publish_state("READY:ACTION_SERVER_AVAILABLE")
            else:
                self.get_logger().error("Foxy /navigate_to_pose action server was lost")
                if self.goal_handle is None:
                    self.publish_state("WAITING:NAV2_ACTION_SERVER")
        self.publish_bridge_ready(server_ready)
        self.publish_goal_active(self.goal_active)
        self.publish_goal_pending(self.goal_pending)
        message = String()
        message.data = self.current_state
        self.state_publisher.publish(message)

    def ready_callback(self, message):
        self.ready_received = time.monotonic()
        if self.mission_token and not message.data:
            self.stop_mission('localization_revoked')
        was_ready = self.localization_ready
        self.localization_ready = bool(message.data)
        if was_ready and not self.localization_ready and (
            self.goal_active or self.goal_handle is not None or self.pending_goal is not None
        ):
            self.request_serial += 1
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.pending_goal = None
            self.pending_serial = None
            self.publish_goal_pending(False)
            self.publish_goal_active(False)
            self.publish_state("CANCELING:LOCALIZATION_NOT_READY")

    def armed_callback(self, message):
        self.armed_received = time.monotonic()
        if self.mission_token:
            if message.data:
                self.mission_had_arm = True
            elif self.mission_had_arm:
                self.stop_mission('gate_disarmed')
        was_armed = self.armed
        self.armed = bool(message.data)
        if self.armed and not was_armed and self.pending_goal is not None:
            message_to_send = self.pending_goal
            serial = self.pending_serial
            self.pending_goal = None
            self.pending_serial = None
            self.publish_goal_pending(False)
            self.send_navigation_goal(serial, message_to_send)
        elif was_armed and not self.armed and (self.goal_active or self.goal_handle is not None):
            self.request_serial += 1
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.publish_goal_active(False)
            self.publish_state("CANCELING:GATE_DISARMED")

    @staticmethod
    def valid_pose(message):
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        return (
            message.header.frame_id == "map"
            and all(math.isfinite(value) for value in values)
            and quaternion_norm > 0.5
        )

    @staticmethod
    def pose_signature(message):
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        return (
            float(message.pose.position.x),
            float(message.pose.position.y),
            yaw,
        )

    @staticmethod
    def angle_delta(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))

    def map_callback(self, message):
        self.map_msg = message

    def voa_cloud_callback(self, message):
        self.voa_cloud = message
        self.voa_cloud_time = time.monotonic()

    def geometry_clearance(self, x, y, yaw):
        """Conservative oriented footprint over all accepted terminal poses."""
        from lite3_patrol_nav.obstacle_safety import rectangle_hits
        from costmap_ground_filter import cloud_xyz
        message = self.map_msg
        if message is None:
            return False, 'map_not_ready'
        try:
            info = message.info
            r, w, h = float(info.resolution), int(info.width), int(info.height)
            p, q = info.origin.position, info.origin.orientation
            if (message.header.frame_id != 'map' or not 0 < r <= .2 or w <= 0 or h <= 0
                    or len(message.data) != w*h or not np.isfinite([p.x,p.y,q.x,q.y,q.z,q.w]).all()
                    or abs(q.x)+abs(q.y) > 1e-6 or abs(q.z*q.z+q.w*q.w-1) > .01):
                return False, 'invalid_map'
            map_yaw = 2*math.atan2(q.z, q.w)
            c, s = math.cos(map_yaw), math.sin(map_yaw)
            rotation = np.array([[c,-s],[s,c]])
            center = (np.array([x,y])-[p.x,p.y]) @ rotation
            # Outside/unknown space is not established terminal clearance.
            radius = math.hypot(*self.goal_geometry)+r/math.sqrt(2)
            cols = np.arange(math.floor((center[0]-radius)/r), math.ceil((center[0]+radius)/r)+1)
            rows = np.arange(math.floor((center[1]-radius)/r), math.ceil((center[1]+radius)/r)+1)
            cc, rr = np.meshgrid(cols, rows)
            world = np.column_stack(((cc.ravel()+.5)*r,(rr.ravel()+.5)*r)) @ rotation.T+[p.x,p.y]
            selected = rectangle_hits(world, (x,y), yaw, self.goal_geometry, r/math.sqrt(2))
            cc, rr = cc.ravel()[selected], rr.ravel()[selected]
            if ((cc < 0)|(cc >= w)|(rr < 0)|(rr >= h)).any():
                return False, 'goal_footprint_outside_map'
            values = np.asarray(message.data).reshape(h,w)[rr,cc]
            if (values < 0).any():
                return False, 'goal_footprint_contains_unknown'
            if (values >= 50).any():
                return False, 'goal_footprint_hits_static_obstacle'
            if self.require_voa_obstacles:
                cloud = self.voa_cloud
                now = time.monotonic()
                if (cloud is None or cloud.header.frame_id != 'map'
                        or now-self.voa_cloud_time > .5):
                    return False, 'voa_obstacles_stale'
                stamp = cloud.header.stamp.sec+cloud.header.stamp.nanosec/1e9
                if not -.05 <= now-stamp <= .5:
                    return False, 'voa_obstacles_stale'
                points = cloud_xyz(cloud)
                if not np.isfinite(points).all():
                    return False, 'invalid_voa_cloud'
                if rectangle_hits(points[:,:2], (x,y), yaw, self.goal_geometry, .05/math.sqrt(2)).any():
                    return False, 'goal_footprint_hits_voa_obstacle'
            return True, ''
        except (ValueError, TypeError, AttributeError, IndexError, KeyError, BufferError, OverflowError):
            return False, 'invalid_goal_geometry_input'

    def goal_clearance(self, x, y, yaw=0.):
        """Reject goals too close to static-map or VOA-camera obstacles."""
        if self.goal_geometry is not None:
            return self.geometry_clearance(x, y, yaw)
        radius = self.goal_clearance_m
        if radius <= 0.0:
            return True, ''
        message = self.map_msg
        if message is None:
            return False, 'map_not_ready'
        info = message.info
        resolution = float(info.resolution)
        width, height = int(info.width), int(info.height)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return False, 'invalid_map'
        col = math.floor((x-origin_x)/resolution)
        row = math.floor((y-origin_y)/resolution)
        if not (0 <= col < width and 0 <= row < height):
            return False, 'goal_outside_map'

        def cell(column, line):
            if 0 <= column < width and 0 <= line < height:
                return int(message.data[line*width+column])
            return -1

        if cell(col, row) != 0:
            return False, 'goal_not_free'
        span = int(math.ceil(radius/resolution))
        for delta_row in range(-span, span+1):
            for delta_col in range(-span, span+1):
                if math.hypot(delta_col*resolution, delta_row*resolution) > radius:
                    continue
                if cell(col+delta_col, row+delta_row) >= 50:
                    return False, 'goal_too_close_to_static_obstacle'
        if not self.require_voa_obstacles:
            return True, ''
        cloud = self.voa_cloud
        if (cloud is None or time.monotonic()-self.voa_cloud_time > .5
                or cloud.header.frame_id != 'map'):
            return False, 'voa_obstacles_stale'
        step = int(cloud.point_step)
        if step < 8 or step % 4 or not cloud.data:
            return False, 'invalid_voa_cloud'
        points = np.frombuffer(cloud.data, dtype=np.float32)
        if points.size % (step//4):
            return False, 'invalid_voa_cloud'
        points = points.reshape(-1, step//4)
        points = points[np.isfinite(points[:, :2]).all(axis=1)]
        if len(points):
            distance = np.hypot(points[:, 0]-x, points[:, 1]-y)
            if float(distance.min()) <= radius:
                return False, 'goal_too_close_to_voa_obstacle'
        return True, ''

    def goal_callback(self, message):
        if self.disarm_required:
            return
        if self.mission_token and not self.mission_dispatch:
            self.get_logger().warn('RViz goal ignored: automatic mission owns the goal bridge')
            return
        if not self.valid_pose(message):
            self.get_logger().error(
                "rejected invalid /patrol/goal_request; frame must be map"
            )
            self.publish_state("REJECTED:INVALID_GOAL")
            return
        if not self.localization_ready:
            self.get_logger().error(
                "rejected /patrol/goal_request: localization is not ready"
            )
            self.publish_state("REJECTED:LOCALIZATION_NOT_READY")
            return
        if not self.action_client.server_is_ready():
            self.get_logger().error(
                "rejected /patrol/goal_request: Nav2 action server unavailable"
            )
            self.publish_state("REJECTED:NAV2_ACTION_UNAVAILABLE")
            return
        if not self.mission_dispatch or self.goal_geometry is not None:
            clearance_ok, clearance_reason = self.goal_clearance(
                float(message.pose.position.x), float(message.pose.position.y),
                self.pose_signature(message)[2])
            if not clearance_ok:
                self.get_logger().warn(
                    "rejected /patrol/goal_request: {} (clearance {:.2f} m)".format(
                        clearance_reason, self.goal_clearance_m))
                self.publish_state("REJECTED:" + clearance_reason.upper())
                return

        now = time.monotonic()
        pose = self.pose_signature(message)
        if self.last_goal_pose is not None and not self.mission_dispatch:
            distance = math.hypot(
                pose[0] - self.last_goal_pose[0],
                pose[1] - self.last_goal_pose[1],
            )
            yaw_difference = abs(self.angle_delta(pose[2], self.last_goal_pose[2]))
            if (
                now - self.last_goal_time <= self.duplicate_goal_window_s
                and distance <= 0.05
                and yaw_difference <= math.radians(5.0)
            ):
                self.get_logger().warn(
                    "ignored duplicate /patrol/goal_request within {:.1f}s: "
                    "distance={:.3f}m yaw={:.1f}deg".format(
                        self.duplicate_goal_window_s,
                        distance,
                        math.degrees(yaw_difference),
                    )
                )
                return
        self.last_goal_pose = pose
        self.last_goal_time = now

        self.request_serial += 1
        serial = self.request_serial
        self.request_path_preview(serial, message)
        if self.armed:
            self.pending_goal = None
            self.pending_serial = None
            self.publish_goal_pending(False)
            self.get_logger().warn(
                "executing bridged goal {} immediately in the existing ARMED "
                "session".format(serial)
            )
            self.send_navigation_goal(serial, message)
        else:
            self.pending_goal = message
            self.pending_serial = serial
            self.publish_goal_pending(True)
            self.publish_state("QUEUED:GOAL_{}:WAITING_FOR_ARM".format(serial))
            self.get_logger().warn(
                "queued bridged goal {}; only the first goal requires ARM".format(
                    serial
                )
            )

    def request_path_preview(self, serial, message):
        if not self.planner_client.server_is_ready():
            self.get_logger().warn(
                "planner action unavailable; goal {} remains queued without preview".format(
                    serial
                )
            )
            return
        goal = ComputePathToPose.Goal()
        # Foxy ComputePathToPose uses the current robot pose implicitly and
        # names the target field "pose" (newer Nav2 releases use "goal").
        goal.pose = message
        goal.planner_id = "GridBased"
        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response_future: self.path_response_callback(
                serial, response_future
            )
        )

    def path_response_callback(self, serial, future):
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().warn("path preview request failed: {}".format(error))
            return
        if not handle.accepted:
            self.get_logger().warn("path preview rejected for goal {}".format(serial))
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda path_future: self.path_result_callback(serial, path_future)
        )

    def path_result_callback(self, serial, future):
        try:
            result = future.result()
            path = result.result.path
        except Exception as error:
            self.get_logger().warn("path preview failed: {}".format(error))
            return
        if serial != self.request_serial:
            return
        # Foxy still supplies an empty Path on an aborted planner result.
        # Do not label that as PATH_READY; the active Nav2 tree may replan.
        if result.status != 4 or not path.poses:  # GoalStatus.STATUS_SUCCEEDED
            if self.pending_serial == serial:
                self.publish_state("QUEUED:GOAL_{}:PREVIEW_NO_PATH".format(serial))
            self.get_logger().warn("no valid path preview for goal {} (status={}, poses={})".format(
                serial, result.status, len(path.poses)))
            return
        self.preview_path_publisher.publish(path)
        if self.pending_serial == serial:
            self.publish_state("QUEUED:GOAL_{}:PATH_READY".format(serial))
        self.get_logger().info(
            "path preview ready for queued goal {} ({} poses)".format(
                serial, len(path.poses)
            )
        )

    def send_navigation_goal(self, serial, message):
        goal = NavigateToPose.Goal()
        goal.pose = message
        goal.behavior_tree = ""
        # Treat the request-in-flight as active so every send failure or Nav2
        # result creates a true -> false edge that cleanly disarms the gate.
        self.publish_goal_active(True)
        self.publish_state("SENDING:GOAL_{}".format(serial))
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response_future: self.goal_response_callback(
                serial, response_future
            )
        )

    def goal_response_callback(self, serial, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error("goal request failed: {}".format(error))
            if serial == self.request_serial:
                self.publish_goal_active(False)
            if serial == self.request_serial:
                self.publish_state("REJECTED:ACTION_REQUEST_FAILED")
            return
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected goal {}".format(serial))
            if serial == self.request_serial:
                self.publish_goal_active(False)
            if serial == self.request_serial:
                self.publish_state("REJECTED:GOAL_{}".format(serial))
            return

        if serial != self.request_serial:
            self.get_logger().warn(
                "canceling late acceptance for superseded goal {}".format(serial)
            )
            goal_handle.cancel_goal_async()
            return

        self.goal_handle = goal_handle
        self.publish_goal_active(True)
        self.get_logger().warn("Nav2 accepted bridged goal {}".format(serial))
        self.publish_state("ACCEPTED:GOAL_{}".format(serial))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future_result: self.result_callback(serial, future_result)
        )

    def result_callback(self, serial, future):
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
        except Exception as error:
            self.get_logger().error("goal result failed: {}".format(error))
            if serial == self.request_serial:
                self.goal_handle = None
                self.publish_goal_active(False)
                self.publish_state("RESULT_ERROR:GOAL_{}".format(serial))
            return
        if serial != self.request_serial:
            self.get_logger().info(
                "ignoring result for superseded goal {} (status {})".format(
                    serial, status
                )
            )
            return
        self.goal_handle = None
        self.publish_goal_active(False)
        self.get_logger().warn(
            "Nav2 bridged goal {} finished with status {}".format(serial, status)
        )
        status_name = {
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
        }.get(status, "STATUS_{}".format(status))
        suffix = ":READY_FOR_NEXT_GOAL" if self.armed else ""
        self.publish_state(
            "RESULT:GOAL_{}:{}{}".format(serial, status_name, suffix)
        )

    def cancel_callback(self, _request, response):
        if self.mission_token:
            self.stop_mission('operator_cancel_or_mission_end')
            response.success = True
            response.message = 'mission canceled; DISARM requested'
            return response
        self.request_serial += 1
        had_pending = self.pending_goal is not None
        self.pending_goal = None
        self.pending_serial = None
        self.publish_goal_pending(False)
        if self.goal_handle is None:
            response.success = True
            response.message = (
                "queued goal canceled" if had_pending else "no active bridged goal"
            )
            self.publish_goal_active(False)
            self.publish_state("CANCELED:NO_ACTIVE_GOAL")
            return response
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.publish_goal_active(False)
        response.success = True
        response.message = "cancel request sent; velocity gate state is unchanged"
        self.publish_state("CANCEL_REQUESTED")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.action_client.destroy()
        node.planner_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
