#!/usr/bin/env python3
"""Build a conservative wide-area Nav2 map from Livox frames and LIO poses.

Unlike prepare_patrol_map.py, this program does not declare a fixed tube around
the driven trajectory to be the whole free space. It reconstructs per-frame
sensor rays, requires repeated ground evidence near every free cell, rejects
steep/rough cells, inflates obstacles, and keeps only free components connected
to the physically traversed route.
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import sqlite3
import sys

import cv2
import numpy as np
import yaml


LIDAR_TOPIC = "/fast_livo2/livox/lidar"
CUSTOM_POINT_DTYPE = np.dtype({
    "names": ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
    "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": 20,
})


def natural_key(path):
    return [int(value) if value.isdigit() else value for value in re.split(r"(\d+)", path)]


def quaternion_matrix(quaternion):
    qx, qy, qz, qw = quaternion
    norm = qx*qx + qy*qy + qz*qz + qw*qw
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    return np.asarray([
        [1-scale*(qy*qy+qz*qz), scale*(qx*qy-qz*qw), scale*(qx*qz+qy*qw)],
        [scale*(qx*qy+qz*qw), 1-scale*(qx*qx+qz*qz), scale*(qy*qz-qx*qw)],
        [scale*(qx*qz-qy*qw), scale*(qy*qz+qx*qw), 1-scale*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def interpolate_pose(stamp, stamps, positions, quaternions):
    upper = int(np.searchsorted(stamps, stamp, side="left"))
    if upper <= 0:
        return positions[0], quaternion_matrix(quaternions[0])
    if upper >= len(stamps):
        return positions[-1], quaternion_matrix(quaternions[-1])
    lower = upper - 1
    span = stamps[upper] - stamps[lower]
    alpha = 0.0 if span <= 0 else float(stamp-stamps[lower]) / float(span)
    position = positions[lower]*(1.0-alpha) + positions[upper]*alpha
    first = quaternions[lower]
    second = quaternions[upper]
    if float(np.dot(first, second)) < 0.0:
        second = -second
    quaternion = first*(1.0-alpha) + second*alpha
    quaternion /= max(float(np.linalg.norm(quaternion)), 1e-12)
    return position, quaternion_matrix(quaternion)


def load_trajectory(path, lidar_start, playback_rate):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append((
                int(row["stamp_ns"]),
                [float(row[name]) for name in ("x_m", "y_m", "z_m")],
                [float(row[name]) for name in ("qx", "qy", "qz", "qw")],
            ))
    if len(rows) < 2:
        raise RuntimeError("trajectory has fewer than two poses")
    playback_start = rows[0][0]
    stamps = np.asarray(
        [lidar_start + int((row[0]-playback_start)*playback_rate) for row in rows],
        dtype=np.int64,
    )
    positions = np.asarray([row[1] for row in rows], dtype=np.float64)
    quaternions = np.asarray([row[2] for row in rows], dtype=np.float64)
    return stamps, positions, quaternions


class LidarBag:
    def __init__(self, bag_dir):
        self.paths = sorted(glob.glob(os.path.join(bag_dir, "*.db3")), key=natural_key)
        if not self.paths:
            raise RuntimeError("no db3 files in {}".format(bag_dir))
        self.entries = []
        self.type_name = None
        for path in self.paths:
            connection = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
            topic = connection.execute(
                "SELECT id,type FROM topics WHERE name=?", (LIDAR_TOPIC,)
            ).fetchone()
            if topic:
                topic_id, type_name = topic
                self.type_name = type_name
                self.entries.extend(
                    (int(stamp), path, int(message_id))
                    for message_id, stamp in connection.execute(
                        "SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                        (topic_id,),
                    )
                )
            connection.close()
        self.entries.sort()
        if not self.entries or not self.type_name:
            raise RuntimeError("{} is absent".format(LIDAR_TOPIC))
        self.connections = {}

    @staticmethod
    def _align_cdr(offset, alignment):
        # CDR field alignment starts after the four-byte encapsulation header.
        return 4 + ((offset-4+alignment-1)//alignment)*alignment

    def points(self, entry):
        _, path, message_id = entry
        if path not in self.connections:
            self.connections[path] = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
        blob = self.connections[path].execute(
            "SELECT data FROM messages WHERE id=?", (message_id,)
        ).fetchone()[0]
        if len(blob) < 56 or bytes(blob[:4]) != b"\x00\x01\x00\x00":
            raise RuntimeError("unsupported CustomMsg CDR encapsulation")
        frame_length = int.from_bytes(blob[12:16], "little")
        offset = self._align_cdr(16+frame_length, 4)
        offset = self._align_cdr(offset, 8)
        offset += 8  # timebase
        point_num = int.from_bytes(blob[offset:offset+4], "little")
        offset += 8  # point_num, lidar_id, and three reserved bytes
        sequence_count = int.from_bytes(blob[offset:offset+4], "little")
        offset += 4
        count = min(point_num, sequence_count)
        required = offset+count*CUSTOM_POINT_DTYPE.itemsize
        # ROS CDR may omit only the final struct's alignment padding byte.
        if len(blob) == required-1:
            blob = bytes(blob)+b"\x00"
        elif len(blob) < required:
            raise RuntimeError(
                "truncated CustomMsg: points={} bytes={} required={}".format(
                    count, len(blob), required
                )
            )
        return np.frombuffer(blob, dtype=CUSTOM_POINT_DTYPE, count=count, offset=offset)

    def close(self):
        for connection in self.connections.values():
            connection.close()


def read_pcd_xyz(path):
    header = {}
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD DATA line is missing")
            decoded = line.decode("ascii", "replace").strip()
            parts = decoded.split()
            if parts and not decoded.startswith("#"):
                header[parts[0]] = parts[1:]
            if parts[:1] == ["DATA"]:
                if parts[1:] != ["binary"]:
                    raise RuntimeError("reference PCD must use binary encoding")
                offset = stream.tell()
                break
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    kinds = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"]*len(fields))]
    if fields[:3] != ["x", "y", "z"]:
        raise RuntimeError("reference PCD must begin with x y z")
    if any(size != 4 or kind != "F" or count != 1 for size, kind, count in zip(sizes, kinds, counts)):
        raise RuntimeError("reference PCD fields must be scalar float32")
    point_count = int(header["POINTS"][0])
    return np.memmap(
        path, mode="r", dtype="<f4", offset=offset, shape=(point_count, len(fields))
    ), point_count


def smooth_track(track, window=11):
    kernel = np.ones(window, dtype=np.float64) / window
    pad = window // 2
    result = np.empty_like(track)
    for axis in range(3):
        values = np.pad(track[:, axis], (pad, pad), mode="edge")
        result[:, axis] = np.convolve(values, kernel, mode="valid")[:len(track)]
    return result


def raster_coordinates(x, y, x_min, y_min, resolution, height):
    col = np.floor((x-x_min)/resolution).astype(np.int32)
    row = height - 1 - np.floor((y-y_min)/resolution).astype(np.int32)
    return row, col


def nearest_track_height(track, shape, x_min, y_min, resolution):
    height, width = shape
    row, col = raster_coordinates(track[:, 0], track[:, 1], x_min, y_min, resolution, height)
    valid = (row >= 0) & (row < height) & (col >= 0) & (col < width)
    row, col, z = row[valid], col[valid], track[valid, 2]
    z_sum = np.zeros(shape, np.float64)
    z_count = np.zeros(shape, np.uint32)
    np.add.at(z_sum, (row, col), z)
    np.add.at(z_count, (row, col), 1)
    seeds = z_count > 0
    seed_z = np.zeros(shape, np.float32)
    seed_z[seeds] = (z_sum[seeds]/z_count[seeds]).astype(np.float32)
    source = np.full(shape, 255, np.uint8)
    source[seeds] = 0
    distance, labels = cv2.distanceTransformWithLabels(
        source, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL
    )
    max_label = int(labels.max())
    values = np.zeros(max_label+1, np.float64)
    counts = np.zeros(max_label+1, np.uint32)
    np.add.at(values, labels[seeds], seed_z[seeds])
    np.add.at(counts, labels[seeds], 1)
    populated = counts > 0
    values[populated] /= counts[populated]
    return values[labels].astype(np.float32), distance, row, col


def increment_unique(grid, row, col):
    if len(row) == 0:
        return 0
    flat = np.unique(row.astype(np.int64)*grid.shape[1] + col.astype(np.int64))
    target = grid.ravel()
    target[flat] = np.minimum(target[flat].astype(np.uint32)+1, 65535).astype(np.uint16)
    return len(flat)


def representative_rays(bin_id, ranges, ground, obstacle):
    selected_ground = np.flatnonzero(ground)
    if len(selected_ground):
        order = np.lexsort((-ranges[selected_ground], bin_id[selected_ground]))
        candidates = selected_ground[order]
        _, first = np.unique(bin_id[candidates], return_index=True)
        selected_ground = candidates[first]
    selected_obstacle = np.flatnonzero(obstacle)
    if len(selected_obstacle):
        order = np.lexsort((ranges[selected_obstacle], bin_id[selected_obstacle]))
        candidates = selected_obstacle[order]
        _, first = np.unique(bin_id[candidates], return_index=True)
        selected_obstacle = candidates[first]
    chosen = {int(bin_id[index]): (int(index), False) for index in selected_ground}
    for index in selected_obstacle:
        key = int(bin_id[index])
        if key not in chosen or ranges[index] < ranges[chosen[key][0]]:
            chosen[key] = (int(index), True)
    return list(chosen.values())


def propagate_values(seed_mask, seed_values):
    source = np.full(seed_mask.shape, 255, np.uint8)
    source[seed_mask] = 0
    _, labels = cv2.distanceTransformWithLabels(
        source, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL
    )
    max_label = int(labels.max())
    sums = np.zeros(max_label+1, np.float64)
    counts = np.zeros(max_label+1, np.uint32)
    np.add.at(sums, labels[seed_mask], seed_values[seed_mask])
    np.add.at(counts, labels[seed_mask], 1)
    populated = counts > 0
    sums[populated] /= counts[populated]
    return sums[labels].astype(np.float32)


def build_local_ground_model(
    pcd_path, baseline_z, shape, x_min, y_min, resolution, sensor_height,
    chunk_points=1_000_000,
):
    points, point_count = read_pcd_xyz(pcd_path)
    # Search generously around the expected ground (-sensor_height relative to
    # the LIO body trajectory). The densest height bin wins; ties prefer the
    # physically expected sensor height rather than a wall return.
    edges = np.arange(-1.20, 0.151, 0.05, dtype=np.float32)
    centers = (edges[:-1]+edges[1:])/2.0
    bin_count = len(centers)
    histogram = np.zeros((shape[0]*shape[1], bin_count), dtype=np.uint16)
    accepted = 0
    for start in range(0, point_count, chunk_points):
        xyz = np.asarray(points[start:start+chunk_points, :3])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        row, col = raster_coordinates(
            xyz[:, 0], xyz[:, 1], x_min, y_min, resolution, shape[0]
        )
        inside = (row >= 0) & (row < shape[0]) & (col >= 0) & (col < shape[1])
        row, col, xyz = row[inside], col[inside], xyz[inside]
        relative = xyz[:, 2]-baseline_z[row, col]
        bin_id = np.floor((relative-edges[0])/0.05).astype(np.int32)
        valid = (bin_id >= 0) & (bin_id < bin_count)
        flat = row[valid].astype(np.int64)*shape[1] + col[valid].astype(np.int64)
        combined = flat*bin_count + bin_id[valid]
        target = histogram.ravel()
        np.add.at(target, combined, 1)
        accepted += int(np.count_nonzero(valid))

    prior_distance = np.abs(centers+sensor_height)
    scores = histogram.astype(np.float32)-prior_distance[None, :]*0.01
    mode = np.argmax(scores, axis=1)
    mode_count = histogram[np.arange(len(histogram)), mode]
    seed = (mode_count > 0).reshape(shape)
    seed_values = np.zeros(shape, np.float32)
    seed_values.ravel()[mode_count > 0] = (
        baseline_z.ravel()[mode_count > 0]+centers[mode[mode_count > 0]]
    )
    propagated = propagate_values(seed, seed_values)
    model = cv2.GaussianBlur(propagated, (5, 5), 0)
    return model, seed, {
        "pcd_points": point_count,
        "pcd_ground_candidates": accepted,
        "pcd_ground_seed_cells": int(np.count_nonzero(seed)),
        "height_bin_m": 0.05,
    }


def write_pgm(path, image):
    with open(path, "wb") as stream:
        stream.write("P5\n{} {}\n255\n".format(image.shape[1], image.shape[0]).encode("ascii"))
        stream.write(image.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir")
    parser.add_argument("trajectory_csv")
    parser.add_argument("output_dir")
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--reference-pcd", default="")
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--max-range", type=float, default=12.0)
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--frame-stride", type=int, default=2)
    parser.add_argument("--point-stride", type=int, default=6)
    parser.add_argument("--angular-bins", type=int, default=360)
    parser.add_argument("--sensor-height", type=float, default=0.47)
    parser.add_argument("--ground-band-below", type=float, default=0.12)
    parser.add_argument("--ground-band-above", type=float, default=0.12)
    parser.add_argument("--min-obstacle-height", type=float, default=0.15)
    parser.add_argument("--max-obstacle-height", type=float, default=1.20)
    parser.add_argument("--min-ground-observations", type=int, default=2)
    parser.add_argument("--min-free-observations", type=int, default=3)
    parser.add_argument("--min-obstacle-observations", type=int, default=3)
    parser.add_argument("--obstacle-ground-ratio", type=float, default=0.50)
    parser.add_argument("--ground-support-radius", type=float, default=0.65)
    parser.add_argument("--max-slope-deg", type=float, default=20.0)
    parser.add_argument("--max-ground-std", type=float, default=0.12)
    parser.add_argument("--obstacle-inflation", type=float, default=0.15)
    parser.add_argument("--free-close-radius", type=float, default=0.20)
    # The recorded body trajectory is known traversable.  Keep enough lateral
    # clearance for Lite3's 0.40 x 0.30 m padded half-footprint in nav2_safe.
    parser.add_argument("--guaranteed-route-radius", type=float, default=0.45)
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    if args.resolution <= 0 or args.max_range <= 1.0 or args.frame_stride < 1 or args.point_stride < 1:
        raise RuntimeError("invalid resolution, range, or stride")
    if os.path.exists(args.output_dir):
        raise RuntimeError("refusing to overwrite {}".format(args.output_dir))
    os.makedirs(args.output_dir)

    bag = LidarBag(args.bag_dir)
    lidar_start = bag.entries[0][0]
    trajectory_stamps, positions, quaternions = load_trajectory(
        args.trajectory_csv, lidar_start, args.playback_rate
    )
    track = smooth_track(positions)
    pad = args.max_range + args.margin
    x_min = math.floor((float(track[:, 0].min())-pad)/args.resolution)*args.resolution
    x_max = math.ceil((float(track[:, 0].max())+pad)/args.resolution)*args.resolution
    y_min = math.floor((float(track[:, 1].min())-pad)/args.resolution)*args.resolution
    y_max = math.ceil((float(track[:, 1].max())+pad)/args.resolution)*args.resolution
    width = int(round((x_max-x_min)/args.resolution))+1
    height = int(round((y_max-y_min)/args.resolution))+1
    shape = (height, width)
    reference_z, distance_to_route_px, track_row, track_col = nearest_track_height(
        track, shape, x_min, y_min, args.resolution
    )
    ground_model_stats = None
    ground_model_seed = np.zeros(shape, dtype=bool)
    classification_ground_z = reference_z
    if args.reference_pcd:
        classification_ground_z, ground_model_seed, ground_model_stats = build_local_ground_model(
            args.reference_pcd, reference_z, shape, x_min, y_min,
            args.resolution, args.sensor_height,
        )

    ground_hits = np.zeros(shape, np.uint16)
    obstacle_hits = np.zeros(shape, np.uint16)
    free_hits = np.zeros(shape, np.uint16)
    ground_point_count = np.zeros(shape, np.uint32)
    ground_z_sum = np.zeros(shape, np.float64)
    ground_z_square_sum = np.zeros(shape, np.float64)
    lidar_to_body = np.asarray([-0.011, -0.02329, 0.04412], dtype=np.float64)
    first_mapped = trajectory_stamps[0]
    last_mapped = trajectory_stamps[-1]
    chosen_entries = bag.entries[::args.frame_stride]
    if args.max_frames > 0:
        chosen_entries = chosen_entries[:args.max_frames]

    totals = {
        "frames_considered": len(chosen_entries), "frames_processed": 0,
        "raw_points": 0, "sampled_points": 0, "valid_points": 0,
        "ground_points": 0, "obstacle_points": 0, "rays": 0,
    }
    relative_z_edges = np.linspace(-1.5, 1.5, 121)
    relative_z_histogram = np.zeros(len(relative_z_edges)-1, dtype=np.int64)
    if args.reference_pcd:
        ground_low = -args.ground_band_below
        ground_high = args.ground_band_above
        obstacle_low = args.min_obstacle_height
        obstacle_high = args.max_obstacle_height
    else:
        ground_low = -args.sensor_height-args.ground_band_below
        ground_high = -args.sensor_height+args.ground_band_above
        obstacle_low = -args.sensor_height+args.min_obstacle_height
        obstacle_high = -args.sensor_height+args.max_obstacle_height

    for sequence, entry in enumerate(chosen_entries, 1):
        stamp = entry[0]
        if stamp < first_mapped or stamp > last_mapped:
            continue
        all_points = bag.points(entry)
        sampled = all_points[::args.point_stride]
        totals["raw_points"] += len(all_points)
        totals["sampled_points"] += len(sampled)
        if len(sampled) == 0:
            continue
        xyz = np.column_stack((sampled["x"], sampled["y"], sampled["z"])).astype(np.float64)
        tags = sampled["tag"]
        ranges = np.linalg.norm(xyz, axis=1)
        valid = (
            np.isfinite(xyz).all(axis=1)
            & (ranges >= 0.8) & (ranges <= args.max_range)
            & (((tags & 0x30) == 0x00) | ((tags & 0x30) == 0x10))
        )
        xyz, ranges = xyz[valid], ranges[valid]
        if len(xyz) == 0:
            continue
        totals["valid_points"] += len(xyz)
        position, rotation = interpolate_pose(stamp, trajectory_stamps, positions, quaternions)
        body_points = xyz + lidar_to_body
        map_points = body_points @ rotation.T + position
        sensor_origin = rotation @ lidar_to_body + position
        row, col = raster_coordinates(
            map_points[:, 0], map_points[:, 1], x_min, y_min, args.resolution, height
        )
        inside = (row >= 0) & (row < height) & (col >= 0) & (col < width)
        row, col = row[inside], col[inside]
        map_points, body_points, ranges = map_points[inside], body_points[inside], ranges[inside]
        if len(map_points) == 0:
            continue
        relative_z = map_points[:, 2]-classification_ground_z[row, col]
        relative_z_histogram += np.histogram(relative_z, bins=relative_z_edges)[0]
        ground = (relative_z >= ground_low) & (relative_z <= ground_high)
        obstacle = (relative_z >= obstacle_low) & (relative_z <= obstacle_high)
        totals["ground_points"] += int(np.count_nonzero(ground))
        totals["obstacle_points"] += int(np.count_nonzero(obstacle))
        increment_unique(ground_hits, row[ground], col[ground])
        increment_unique(obstacle_hits, row[obstacle], col[obstacle])
        np.add.at(ground_point_count, (row[ground], col[ground]), 1)
        np.add.at(ground_z_sum, (row[ground], col[ground]), map_points[ground, 2])
        np.add.at(ground_z_square_sum, (row[ground], col[ground]), map_points[ground, 2]**2)

        angles = np.arctan2(body_points[:, 1], body_points[:, 0])
        bins = np.floor((angles+math.pi)*args.angular_bins/(2.0*math.pi)).astype(np.int32)
        bins = np.clip(bins, 0, args.angular_bins-1)
        frame_free = np.zeros(shape, np.uint8)
        origin_row, origin_col = raster_coordinates(
            np.asarray([sensor_origin[0]]), np.asarray([sensor_origin[1]]),
            x_min, y_min, args.resolution, height,
        )
        origin_pixel = (int(origin_col[0]), int(origin_row[0]))
        rays = representative_rays(bins, ranges, ground, obstacle)
        for point_index, is_obstacle in rays:
            endpoint = (int(col[point_index]), int(row[point_index]))
            cv2.line(frame_free, origin_pixel, endpoint, 255, 1, cv2.LINE_8)
            if is_obstacle:
                frame_free[endpoint[1], endpoint[0]] = 0
        increment_unique(free_hits, *np.nonzero(frame_free))
        totals["rays"] += len(rays)
        totals["frames_processed"] += 1
        if sequence == 1 or sequence % 100 == 0 or sequence == len(chosen_entries):
            print(
                "frames {}/{} processed={} ground={} obstacle={} rays={}".format(
                    sequence, len(chosen_entries), totals["frames_processed"],
                    totals["ground_points"], totals["obstacle_points"], totals["rays"],
                ),
                flush=True,
            )
    bag.close()

    observed_ground = ground_hits >= args.min_ground_observations
    support_radius_px = max(1, int(math.ceil(args.ground_support_radius/args.resolution)))
    support_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2*support_radius_px+1, 2*support_radius_px+1)
    )
    ground_support = cv2.dilate(observed_ground.astype(np.uint8), support_kernel) > 0
    ray_free = free_hits >= args.min_free_observations

    ground_mean = np.zeros(shape, np.float32)
    populated = ground_point_count > 0
    ground_mean[populated] = (ground_z_sum[populated]/ground_point_count[populated]).astype(np.float32)
    propagated_ground = propagate_values(observed_ground, ground_mean)
    smooth_ground = cv2.GaussianBlur(propagated_ground, (5, 5), 0)
    gradient_x = cv2.Sobel(smooth_ground, cv2.CV_32F, 1, 0, ksize=3)/(8.0*args.resolution)
    gradient_y = cv2.Sobel(smooth_ground, cv2.CV_32F, 0, 1, ksize=3)/(8.0*args.resolution)
    slope_deg = np.degrees(np.arctan(np.hypot(gradient_x, gradient_y)))
    variance = np.zeros(shape, np.float64)
    variance[populated] = np.maximum(
        ground_z_square_sum[populated]/ground_point_count[populated]
        - (ground_z_sum[populated]/ground_point_count[populated])**2,
        0.0,
    )
    ground_std = np.sqrt(variance)
    rough_seed = populated & (ground_std > args.max_ground_std)
    rough = cv2.dilate(rough_seed.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0

    obstacle_threshold = np.maximum(
        args.min_obstacle_observations,
        np.ceil(ground_hits.astype(np.float32)*args.obstacle_ground_ratio),
    )
    obstacle_seed = obstacle_hits >= obstacle_threshold
    inflation_px = max(1, int(math.ceil(args.obstacle_inflation/args.resolution)))
    inflation_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2*inflation_px+1, 2*inflation_px+1)
    )
    inflated_obstacle = cv2.dilate(obstacle_seed.astype(np.uint8), inflation_kernel) > 0
    candidate_free = (
        ray_free & ground_support & (slope_deg <= args.max_slope_deg) & ~rough & ~inflated_obstacle
    )
    close_px = max(0, int(math.ceil(args.free_close_radius/args.resolution)))
    if close_px > 0:
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2*close_px+1, 2*close_px+1)
        )
        candidate_free = cv2.morphologyEx(
            candidate_free.astype(np.uint8), cv2.MORPH_CLOSE, close_kernel
        ) > 0
        candidate_free &= ~inflated_obstacle

    route_radius_px = args.guaranteed_route_radius/args.resolution
    route_corridor = distance_to_route_px <= route_radius_px
    candidate_free |= route_corridor
    components, labels = cv2.connectedComponents(candidate_free.astype(np.uint8), connectivity=8)
    route_labels = np.unique(labels[route_corridor & (labels > 0)])
    connected_free = np.isin(labels, route_labels) if len(route_labels) else route_corridor

    obstacle_near_free = cv2.dilate(connected_free.astype(np.uint8), support_kernel) > 0
    occupied = inflated_obstacle & obstacle_near_free & ~route_corridor
    grid = np.full(shape, 205, np.uint8)
    grid[connected_free] = 254
    grid[occupied] = 0
    grid[route_corridor] = 254

    map_pgm = os.path.join(args.output_dir, "patrol_map_area.pgm")
    map_yaml = os.path.join(args.output_dir, "patrol_map_area.yaml")
    write_pgm(map_pgm, grid)
    with open(map_yaml, "w", encoding="utf-8") as stream:
        yaml.safe_dump({
            "image": "patrol_map_area.pgm", "mode": "trinary",
            "resolution": args.resolution, "origin": [x_min, y_min, 0.0],
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25,
        }, stream, sort_keys=False)

    preview = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    path_pixels = np.column_stack([track_col, track_row]).astype(np.int32)
    cv2.polylines(preview, [path_pixels.reshape(-1, 1, 2)], False, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(os.path.join(args.output_dir, "patrol_map_area_preview.png"), preview)

    diagnostics = np.zeros((height, width, 3), np.uint8)
    diagnostics[:] = (70, 70, 70)
    diagnostics[ray_free] = (150, 90, 40)
    diagnostics[ground_support] = (180, 180, 40)
    diagnostics[observed_ground] = (50, 210, 50)
    diagnostics[inflated_obstacle] = (20, 20, 230)
    diagnostics[connected_free] = (240, 240, 240)
    cv2.polylines(diagnostics, [path_pixels.reshape(-1, 1, 2)], False, (255, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(os.path.join(args.output_dir, "traversability_diagnostics.png"), diagnostics)

    np.savez_compressed(
        os.path.join(args.output_dir, "traversability_evidence.npz"),
        ground_hits=ground_hits, obstacle_hits=obstacle_hits, free_hits=free_hits,
        ground_point_count=ground_point_count, ground_mean=ground_mean,
        ground_std=ground_std.astype(np.float32), slope_deg=slope_deg.astype(np.float32),
        reference_z=reference_z, distance_to_route_px=distance_to_route_px,
        classification_ground_z=classification_ground_z,
        ground_model_seed=ground_model_seed,
    )

    summary = {
        "source_bag": os.path.abspath(args.bag_dir),
        "trajectory_csv": os.path.abspath(args.trajectory_csv),
        "lidar_messages": len(bag.entries),
        "totals": totals,
        "bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "grid": {"width": width, "height": height, "resolution_m": args.resolution},
        "cells": {
            "observed_ground": int(np.count_nonzero(observed_ground)),
            "ray_free": int(np.count_nonzero(ray_free)),
            "candidate_free_before_component_filter": int(np.count_nonzero(candidate_free)),
            "connected_free": int(np.count_nonzero(connected_free)),
            "occupied": int(np.count_nonzero(occupied)),
            "unknown": int(np.count_nonzero(grid == 205)),
            "rough": int(np.count_nonzero(rough)),
            "slope_rejected": int(np.count_nonzero(ground_support & (slope_deg > args.max_slope_deg))),
        },
        "components_before_route_filter": int(components-1),
        "ground_model": ground_model_stats,
        "relative_z_histogram": [
            {
                "center_m": round(float((relative_z_edges[index]+relative_z_edges[index+1])/2.0), 4),
                "count": int(count),
            }
            for index, count in enumerate(relative_z_histogram)
        ],
        "parameters": vars(args),
    }
    with open(os.path.join(args.output_dir, "traversability_summary.json"), "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
        stream.write("\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
