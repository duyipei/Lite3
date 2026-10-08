#!/usr/bin/env python3
"""Offline neighbor-frame validation for an RGB-D visual map database."""

import argparse
import json
import math
import sqlite3

import cv2
import numpy as np


def yaw(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def angle(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def appearance_vector(image_bgr):
    small = cv2.resize(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY), (32, 18), interpolation=cv2.INTER_AREA).astype(np.float32)
    small -= float(small.mean())
    return (small/max(float(np.linalg.norm(small)), 1e-6)).ravel()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("--queries", type=int, default=12)
    args = parser.parse_args()
    connection = sqlite3.connect("file:{}?mode=ro".format(args.database), uri=True)
    meta = {key: json.loads(value) for key, value in connection.execute("SELECT key,value FROM metadata")}
    k = np.asarray(meta["color_k"], np.float64)
    t_base_camera = np.asarray(meta["base_to_color_optical"], np.float64)
    frames = []
    for identifier, pose_blob, points_blob, descriptor_blob, count, jpeg_blob in connection.execute(
        "SELECT id,map_camera,points,descriptors,feature_count,color_jpeg FROM keyframes ORDER BY id"
    ):
        image = cv2.imdecode(np.frombuffer(jpeg_blob, np.uint8), cv2.IMREAD_COLOR)
        histogram = cv2.calcHist([cv2.cvtColor(image, cv2.COLOR_BGR2HSV)], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(histogram, histogram)
        keep = min(count, 600)
        frames.append({
            "id": identifier,
            "map_camera": np.frombuffer(pose_blob, "<f8").reshape(4, 4).copy(),
            "points": np.frombuffer(points_blob, "<f4").reshape(count, 3)[:keep].copy(),
            "descriptors": np.frombuffer(descriptor_blob, np.uint8).reshape(count, 32)[:keep].copy(),
            "image": image,
            "histogram": histogram,
            "appearance": appearance_vector(image),
        })
    connection.close()
    orb = cv2.ORB_create(nfeatures=900, scaleFactor=1.2, nlevels=8, fastThreshold=12)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    query_ids = np.linspace(0, len(frames)-1, min(args.queries, len(frames)), dtype=int)
    results = []
    for query_id in query_ids:
        query = frames[query_id]
        keypoints, descriptors = orb.detectAndCompute(cv2.cvtColor(query["image"], cv2.COLOR_BGR2GRAY), None)
        candidates = sorted(
            (frame for frame in frames if frame["id"] != query["id"]),
            key=lambda frame: 0.25*cv2.compareHist(query["histogram"], frame["histogram"], cv2.HISTCMP_BHATTACHARYYA) + 0.75*(1.0-float(np.dot(query["appearance"], frame["appearance"]))),
        )[:8]
        solutions = []
        for frame in candidates:
            pairs = matcher.knnMatch(frame["descriptors"], descriptors, k=2)
            matches = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < 0.75*pair[1].distance]
            if len(matches) < 18:
                continue
            object_points = np.asarray([frame["points"][match.queryIdx] for match in matches], np.float32)
            image_points = np.asarray([keypoints[match.trainIdx].pt for match in matches], np.float32)
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_points, image_points, k, None, iterationsCount=150,
                reprojectionError=4.0, confidence=0.995, flags=cv2.SOLVEPNP_EPNP,
            )
            if not ok or inliers is None or len(inliers) < 24:
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            t_current_key = np.eye(4)
            t_current_key[:3, :3] = rotation
            t_current_key[:3, 3] = tvec[:, 0]
            estimated_base = frame["map_camera"] @ np.linalg.inv(t_current_key) @ np.linalg.inv(t_base_camera)
            solutions.append((len(inliers), frame["id"], estimated_base))
        expected_base = query["map_camera"] @ np.linalg.inv(t_base_camera)
        if solutions:
            inliers, matched_id, estimate = max(solutions, key=lambda value: value[0])
            position_error = float(np.linalg.norm(estimate[:2, 3]-expected_base[:2, 3]))
            yaw_error = angle(yaw(estimate), yaw(expected_base))
            result = {"query": int(query_id), "match": int(matched_id), "inliers": int(inliers), "position_error_m": position_error, "yaw_error_deg": math.degrees(yaw_error)}
        else:
            result = {"query": int(query_id), "match": None}
        results.append(result)
        print(json.dumps(result), flush=True)
    successes = [item for item in results if item["match"] is not None]
    acceptable = [item for item in successes if item["position_error_m"] <= 1.0 and item["yaw_error_deg"] <= 20.0]
    summary = {
        "queries": len(results), "solutions": len(successes), "acceptable": len(acceptable),
        "median_position_error_m": float(np.median([item["position_error_m"] for item in successes])) if successes else None,
        "median_yaw_error_deg": float(np.median([item["yaw_error_deg"] for item in successes])) if successes else None,
    }
    print(json.dumps(summary, indent=2))
    return 0 if len(acceptable) >= max(3, len(results)//2) else 2


if __name__ == "__main__":
    raise SystemExit(main())
