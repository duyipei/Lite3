"""Detect accumulated map drift while independent leg odometry stays still."""
import math


def registration_seed(pose, offset_xyz, offset_rpy):
    """Compose T_map_visual_reference * T_visual_reference_registration.

    A sensor/reference-frame offset is multiplied on the right, so its
    translation rotates with the current heading. Map-to-map transforms are
    applied separately by the caller. Returns (x, y, z, yaw), quaternion xyzw.
    """
    if len(pose) != 4 or len(offset_xyz) != 3 or len(offset_rpy) != 3:
        raise ValueError('Expected an xyz/yaw pose and xyz/rpy reference offset')
    if not all(math.isfinite(v) for v in (*pose, *offset_xyz, *offset_rpy)):
        raise ValueError('Registration seed transform must be finite')
    x, y, z, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    roll, pitch, heading = offset_rpy
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(heading/2), math.sin(heading/2)
    qx, qy = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy
    qz, qw = cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
    cz, sz = math.cos(yaw/2), math.sin(yaw/2)
    q = (cz*qx-sz*qy, cz*qy+sz*qx, cz*qz+sz*qw, cz*qw-sz*qz)
    heading = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
    return (x+c*offset_xyz[0]-s*offset_xyz[1],
            y+s*offset_xyz[0]+c*offset_xyz[1], z+offset_xyz[2], heading), q


def angle_difference(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


class StationaryDriftGuard:
    def __init__(self, position_limit=.20, yaw_limit=.15,
                 leg_position_span=.05, leg_yaw_span=.08):
        self.position_limit, self.yaw_limit = position_limit, yaw_limit
        self.leg_position_span, self.leg_yaw_span = leg_position_span, leg_yaw_span
        self.reset()

    def reset(self):
        self.anchor = None

    def observe(self, map_pose, leg_pose):
        if not all(math.isfinite(v) for v in (*map_pose, *leg_pose)):
            return 'stationary_guard_invalid_pose'
        if self.anchor is None:
            self.anchor = (map_pose, leg_pose)
            return None
        map_start, leg_start = self.anchor
        # Real translation or turning ends the stationary interval. There is
        # no fixed map/leg anchor across a patrol: normal moving NDT corrections
        # are still supervised by the existing per-scan and liveness guards.
        if (math.hypot(leg_pose[0]-leg_start[0], leg_pose[1]-leg_start[1])
                > self.leg_position_span or
                angle_difference(leg_pose[2], leg_start[2]) > self.leg_yaw_span):
            self.anchor = (map_pose, leg_pose)
            return None
        distance = math.hypot(map_pose[0]-map_start[0], map_pose[1]-map_start[1])
        yaw = angle_difference(map_pose[2], map_start[2])
        if distance > self.position_limit or yaw > self.yaw_limit:
            return 'stationary_map_drift_{:.3f}m_{:.3f}rad'.format(distance, yaw)
        return None
