#!/usr/bin/env python3
"""Build an ORB RGB-D keyframe database aligned to a FAST-LIVO map.

The source bag is never modified. FAST-LIVO odometry in this project was
recorded with wall-clock stamps during slowed playback, so trajectory time is
mapped back to bag time using the recorded playback rate.
"""

import argparse
import bisect
import csv
import glob
import json
import math
import os
import re
import sqlite3
import struct
import sys
from types import SimpleNamespace

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPIC_COLOR = "/camera/color/image_raw"
TOPIC_DEPTH = "/camera/depth/image_rect_raw"
TOPIC_COLOR_INFO = "/camera/color/camera_info"
TOPIC_DEPTH_INFO = "/camera/depth/camera_info"
TOPIC_EXTRINSICS = "/camera/extrinsics/depth_to_color"
TOPIC_LIDAR = "/fast_livo2/livox/lidar"


def deserialize_realsense_extrinsics(blob):
    """Decode Extrinsics when its generated ROS Python package is unavailable.

    realsense2_camera_msgs/msg/Extrinsics contains a std_msgs/Header followed
    by float64[9] rotation and float64[3] translation.  CDR alignment is
    relative to the payload after the four-byte encapsulation header.
    """
    if len(blob) < 4 or blob[0:2] not in (b"\x00\x00", b"\x00\x01"):
        raise RuntimeError("unsupported CDR encapsulation for RealSense extrinsics")
    endian = "<" if blob[1] == 1 else ">"
    offset = 4
    # builtin_interfaces/Time: int32 sec, uint32 nanosec.
    offset += 8
    if offset + 4 > len(blob):
        raise RuntimeError("truncated RealSense extrinsics header")
    frame_length = struct.unpack_from(endian + "I", blob, offset)[0]
    offset += 4 + frame_length
    offset += (-(offset - 4)) % 8
    if offset + 12 * 8 > len(blob):
        raise RuntimeError("truncated RealSense extrinsics values")
    values = struct.unpack_from(endian + "12d", blob, offset)
    return SimpleNamespace(rotation=values[:9], translation=values[9:])


def natural_key(path):
    return [int(value) if value.isdigit() else value for value in re.split(r"(\d+)", path)]


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def quaternion_matrix(qx, qy, qz, qw):
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.asarray([
        [1-s*(qy*qy+qz*qz), s*(qx*qy-qz*qw), s*(qx*qz+qy*qw)],
        [s*(qx*qy+qz*qw), 1-s*(qx*qx+qz*qz), s*(qy*qz-qx*qw)],
        [s*(qx*qz-qy*qw), s*(qy*qz+qx*qw), 1-s*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def transform(rotation, translation):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def load_trajectory(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append({
                "stamp": int(row["stamp_ns"]),
                "xyz": np.asarray([float(row["x_m"]), float(row["y_m"]), float(row["z_m"])], dtype=np.float64),
                "quat": tuple(float(row[name]) for name in ("qx", "qy", "qz", "qw")),
                "yaw": math.radians(float(row["yaw_deg"])),
            })
    if len(rows) < 2:
        raise RuntimeError("trajectory has fewer than two poses")
    return rows


class BagIndex:
    def __init__(self, bag_dir):
        self.paths = sorted(glob.glob(os.path.join(bag_dir, "*.db3")), key=natural_key)
        if not self.paths:
            raise RuntimeError("no .db3 files in {}".format(bag_dir))
        self.connections = {}
        self.entries = {}
        self.types = {}
        wanted = {TOPIC_COLOR, TOPIC_DEPTH, TOPIC_COLOR_INFO, TOPIC_DEPTH_INFO, TOPIC_EXTRINSICS, TOPIC_LIDAR}
        for path in self.paths:
            connection = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
            topic_rows = connection.execute("SELECT id,name,type FROM topics").fetchall()
            for topic_id, name, type_name in topic_rows:
                if name not in wanted:
                    continue
                self.types[name] = type_name
                self.entries.setdefault(name, []).extend(
                    (int(stamp), path, int(message_id))
                    for message_id, stamp in connection.execute(
                        "SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,)
                    )
                )
            connection.close()
        for name in wanted:
            self.entries[name] = sorted(self.entries.get(name, []))
            if not self.entries[name]:
                raise RuntimeError("required topic is absent: {}".format(name))

    def nearest(self, topic, stamp):
        entries = self.entries[topic]
        stamps = [item[0] for item in entries]
        index = bisect.bisect_left(stamps, stamp)
        candidates = entries[max(0, index-1):min(len(entries), index+1)]
        return min(candidates, key=lambda item: abs(item[0]-stamp))

    def message(self, topic, entry):
        _, path, message_id = entry
        if path not in self.connections:
            self.connections[path] = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
        blob = self.connections[path].execute("SELECT data FROM messages WHERE id=?", (message_id,)).fetchone()[0]
        try:
            message_type = get_message(self.types[topic])
        except ModuleNotFoundError:
            if topic == TOPIC_EXTRINSICS and self.types[topic] == "realsense2_camera_msgs/msg/Extrinsics":
                return deserialize_realsense_extrinsics(blob)
            raise
        return deserialize_message(blob, message_type)

    def close(self):
        for connection in self.connections.values():
            connection.close()


def select_keyframes(trajectory, distance_m, yaw_rad):
    selected = [0]
    anchor = trajectory[0]
    for index, pose in enumerate(trajectory[1:], 1):
        distance = float(np.linalg.norm(pose["xyz"][:2] - anchor["xyz"][:2]))
        yaw_delta = abs(math.atan2(math.sin(pose["yaw"]-anchor["yaw"]), math.cos(pose["yaw"]-anchor["yaw"])))
        if distance >= distance_m or yaw_delta >= yaw_rad:
            selected.append(index)
            anchor = pose
    if selected[-1] != len(trajectory)-1:
        selected.append(len(trajectory)-1)
    return selected


def image_array(message):
    if message.encoding.lower() == "rgb8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)[:, :message.width*3].reshape(message.height, message.width, 3)
    if message.encoding.lower() == "bgr8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)[:, :message.width*3].reshape(message.height, message.width, 3)[:, :, ::-1]
    if message.encoding.lower() in ("16uc1", "mono16"):
        return np.frombuffer(message.data, dtype="<u2").reshape(message.height, message.step//2)[:, :message.width]
    raise RuntimeError("unsupported image encoding: {}".format(message.encoding))


def align_depth(depth, kd, kc, rotation, translation):
    v, u = np.nonzero(depth)
    z = depth[v, u].astype(np.float64) * 0.001
    valid = (z > 0.2) & (z < 8.0)
    u, v, z = u[valid], v[valid], z[valid]
    xyz_d = np.column_stack(((u-kd[0, 2])*z/kd[0, 0], (v-kd[1, 2])*z/kd[1, 1], z))
    xyz_c = (rotation @ xyz_d.T).T + translation
    zc = xyz_c[:, 2]
    valid = zc > 0.1
    xyz_c, zc = xyz_c[valid], zc[valid]
    uc = np.rint(kc[0, 0]*xyz_c[:, 0]/zc + kc[0, 2]).astype(np.int32)
    vc = np.rint(kc[1, 1]*xyz_c[:, 1]/zc + kc[1, 2]).astype(np.int32)
    valid = (uc >= 0) & (uc < depth.shape[1]) & (vc >= 0) & (vc < depth.shape[0])
    uc, vc, zc = uc[valid], vc[valid], zc[valid]
    aligned = np.full(depth.shape, np.inf, dtype=np.float32)
    np.minimum.at(aligned.ravel(), vc*depth.shape[1]+uc, zc.astype(np.float32))
    aligned[~np.isfinite(aligned)] = 0.0
    return aligned


def feature_points(color_rgb, aligned_depth, kc, orb):
    gray = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    if descriptors is None:
        return np.empty((0, 2), np.float32), np.empty((0, 3), np.float32), np.empty((0, 32), np.uint8)
    pixels, points, kept = [], [], []
    height, width = aligned_depth.shape
    for index, keypoint in enumerate(keypoints):
        u, v = int(round(keypoint.pt[0])), int(round(keypoint.pt[1]))
        if not (1 <= u < width-1 and 1 <= v < height-1):
            continue
        values = aligned_depth[v-1:v+2, u-1:u+2]
        values = values[values > 0]
        if not len(values):
            continue
        z = float(np.median(values))
        x = (keypoint.pt[0]-kc[0, 2])*z/kc[0, 0]
        y = (keypoint.pt[1]-kc[1, 2])*z/kc[1, 1]
        pixels.append(keypoint.pt)
        points.append((x, y, z))
        kept.append(descriptors[index])
    return np.asarray(pixels, np.float32), np.asarray(points, np.float32), np.asarray(kept, np.uint8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir")
    parser.add_argument("trajectory_csv")
    parser.add_argument("output_db")
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--keyframe-distance", type=float, default=1.5)
    parser.add_argument("--keyframe-yaw-deg", type=float, default=18.0)
    parser.add_argument(
        "--startup-keyframes", type=int, default=6,
        help="mark the first N stored keyframes as the preferred start-area references",
    )
    parser.add_argument(
        "--startup-interval-s", type=float, default=1.0,
        help="time spacing for stationary start-area keyframes",
    )
    parser.add_argument(
        "--startup-region-radius", type=float, default=1.0,
        help="include loop-return keyframes within this radius as start-area appearance references",
    )
    parser.add_argument("--max-pair-gap", type=float, default=0.08)
    parser.add_argument("--base-camera-xyz", nargs=3, type=float, default=(0.25489, 0.0, 0.07249))
    parser.add_argument("--base-camera-rpy-deg", nargs=3, type=float, default=(0.0, 20.0, 0.0))
    args = parser.parse_args()
    selection = os.path.join(os.path.dirname(os.path.abspath(args.bag_dir)), 'camera_selection.yaml')
    if os.path.isfile(selection):
        with open(selection) as stream:
            camera2_selected = 'camera_id: 2' in stream.read().splitlines()
        if camera2_selected:
            raise RuntimeError('This run selects camera2. This builder only supports D435i RGB-D; camera2 calibration and a camera2 3D feature-map builder are required. No database generated.')
    if os.path.exists(args.output_db):
        raise RuntimeError("refusing to overwrite {}".format(args.output_db))

    trajectory = load_trajectory(args.trajectory_csv)
    bag = BagIndex(args.bag_dir)
    lidar_start = bag.entries[TOPIC_LIDAR][0][0]
    lidar_end = bag.entries[TOPIC_LIDAR][-1][0]
    trajectory_start = trajectory[0]["stamp"]
    for pose in trajectory:
        pose["bag_stamp"] = int(lidar_start + (pose["stamp"]-trajectory_start)*args.playback_rate)

    color_info = bag.message(TOPIC_COLOR_INFO, bag.entries[TOPIC_COLOR_INFO][0])
    depth_info = bag.message(TOPIC_DEPTH_INFO, bag.entries[TOPIC_DEPTH_INFO][0])
    extrinsics = bag.message(TOPIC_EXTRINSICS, bag.entries[TOPIC_EXTRINSICS][0])
    kc = np.asarray(color_info.k, dtype=np.float64).reshape(3, 3)
    kd = np.asarray(depth_info.k, dtype=np.float64).reshape(3, 3)
    r_dc = np.asarray(extrinsics.rotation, dtype=np.float64).reshape(3, 3)
    t_dc = np.asarray(extrinsics.translation, dtype=np.float64)

    rpy = np.radians(np.asarray(args.base_camera_rpy_deg, dtype=np.float64))
    # camera_link x-forward/y-left/z-up from color optical x-right/y-down/z-forward.
    r_link_optical = np.asarray([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
    t_base_optical = transform(rpy_matrix(*rpy) @ r_link_optical, np.asarray(args.base_camera_xyz))

    output = sqlite3.connect(args.output_db)
    output.execute("CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL)")
    output.execute("CREATE TABLE keyframes(id INTEGER PRIMARY KEY,bag_stamp INTEGER,map_camera BLOB NOT NULL,keypoints BLOB NOT NULL,points BLOB NOT NULL,descriptors BLOB NOT NULL,feature_count INTEGER NOT NULL,color_jpeg BLOB)")
    orb = cv2.ORB_create(nfeatures=1800, scaleFactor=1.2, nlevels=8, fastThreshold=12)
    selected_motion = select_keyframes(
        trajectory, args.keyframe_distance, math.radians(args.keyframe_yaw_deg)
    )
    # The robot is deliberately held still at the beginning of acquisition.
    # Distance/yaw selection alone would keep only the very first trajectory
    # sample, which can predate the first synchronized RGB-D pair.  Add a
    # short temporal burst from the common sensor interval so a reboot at the
    # physical start has several real reference views.
    pose_stamps = [pose["bag_stamp"] for pose in trajectory]
    common_start = max(
        lidar_start,
        bag.entries[TOPIC_COLOR][0][0],
        bag.entries[TOPIC_DEPTH][0][0],
    )
    startup_indices = []
    startup_count = max(1, int(args.startup_keyframes))
    startup_interval_ns = max(1, int(args.startup_interval_s * 1.0e9))
    for number in range(startup_count):
        target = common_start + number * startup_interval_ns
        index = bisect.bisect_left(pose_stamps, target)
        candidates = range(max(0, index - 1), min(len(trajectory), index + 2))
        startup_indices.append(min(candidates, key=lambda value: abs(pose_stamps[value] - target)))
    startup_index_set = set(startup_indices)
    selected = sorted(set(selected_motion).union(startup_index_set))
    stored = 0
    stored_startup_ids = []
    stored_base_positions = []
    max_gap_ns = int(args.max_pair_gap*1e9)
    for sequence, index in enumerate(selected):
        pose = trajectory[index]
        color_entry = bag.nearest(TOPIC_COLOR, pose["bag_stamp"])
        depth_entry = bag.nearest(TOPIC_DEPTH, pose["bag_stamp"])
        if max(abs(color_entry[0]-pose["bag_stamp"]), abs(depth_entry[0]-pose["bag_stamp"]), abs(color_entry[0]-depth_entry[0])) > max_gap_ns:
            continue
        color = image_array(bag.message(TOPIC_COLOR, color_entry))
        depth = image_array(bag.message(TOPIC_DEPTH, depth_entry))
        aligned = align_depth(depth, kd, kc, r_dc, t_dc)
        pixels, points, descriptors = feature_points(color, aligned, kc, orb)
        if len(points) < 80:
            continue
        t_map_base = transform(quaternion_matrix(*pose["quat"]), pose["xyz"])
        t_map_camera = t_map_base @ t_base_optical
        jpeg_ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(color, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 70])
        output.execute(
            "INSERT INTO keyframes VALUES(?,?,?,?,?,?,?,?)",
            (stored, pose["bag_stamp"], t_map_camera.astype("<f8").tobytes(), pixels.astype("<f4").tobytes(), points.astype("<f4").tobytes(), descriptors.tobytes(), len(points), jpeg.tobytes() if jpeg_ok else None),
        )
        if index in startup_index_set:
            stored_startup_ids.append(stored)
        stored_base_positions.append(t_map_base[:3, 3].copy())
        stored += 1
        print("keyframe {}/{} stored={} features={}".format(sequence+1, len(selected), stored, len(points)), flush=True)

    mapped_end = trajectory[-1]["bag_stamp"]
    startup_pose = trajectory[0]
    startup_region_ids = [
        identifier
        for identifier, position in enumerate(stored_base_positions)
        if float(np.linalg.norm(position[:2] - startup_pose["xyz"][:2]))
        <= max(0.1, args.startup_region_radius)
    ]
    metadata = {
        "format_version": 2,
        "map_frame": "visual_map",
        "source_bag": os.path.abspath(args.bag_dir),
        "trajectory_csv": os.path.abspath(args.trajectory_csv),
        "playback_rate": args.playback_rate,
        "lidar_bag_start_ns": lidar_start,
        "lidar_bag_end_ns": lidar_end,
        "mapped_trajectory_end_ns": mapped_end,
        "unmapped_tail_s": (lidar_end-mapped_end)/1e9,
        "color_k": kc.tolist(),
        "depth_k": kd.tolist(),
        "depth_to_color_rotation": r_dc.tolist(),
        "depth_to_color_translation_m": t_dc.tolist(),
        "base_to_color_optical": t_base_optical.tolist(),
        "keyframe_count": stored,
        "candidate_keyframe_count": len(selected),
        "motion_candidate_keyframe_count": len(selected_motion),
        "startup_keyframe_ids": stored_startup_ids,
        "startup_region_keyframe_ids": startup_region_ids,
        "startup_pose_map_base": [
            float(startup_pose["xyz"][0]),
            float(startup_pose["xyz"][1]),
            float(startup_pose["xyz"][2]),
            float(startup_pose["yaw"]),
        ],
        "startup_interval_s": args.startup_interval_s,
        "startup_region_radius_m": args.startup_region_radius,
    }
    for key, value in metadata.items():
        output.execute("INSERT INTO metadata VALUES(?,?)", (key, json.dumps(value)))
    output.commit()
    integrity = output.execute("PRAGMA integrity_check").fetchone()[0]
    output.close()
    bag.close()
    print(json.dumps(metadata, indent=2))
    print("integrity={}".format(integrity))
    if stored < 10 or integrity != "ok":
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
