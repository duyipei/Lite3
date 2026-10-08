#!/usr/bin/env python3
"""Render lightweight top/side views of a binary FAST-LIVO PCD map."""

import argparse
import csv
import json
import math
import os

import cv2
import numpy as np


def pcd_memmap(path, target_points=2_000_000):
    header = {}
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD DATA line is absent")
            decoded = line.decode("ascii", "replace").strip()
            parts = decoded.split()
            if parts and not decoded.startswith("#"):
                header[parts[0]] = parts[1:]
            if parts[:1] == ["DATA"]:
                if parts[1:] != ["binary"]:
                    raise RuntimeError("only binary PCD is supported")
                offset = stream.tell()
                break
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if any(size != 4 or kind != "F" or count != 1 for size, kind, count in zip(sizes, types, counts)):
        raise RuntimeError("unexpected PCD field layout")
    points = int(header["POINTS"][0])
    raw = np.memmap(path, mode="r", dtype="<f4", offset=offset, shape=(points, len(fields)))
    stride = max(1, int(math.ceil(points / target_points)))
    sampled = np.asarray(raw[::stride, :3])
    sampled = sampled[np.isfinite(sampled).all(axis=1)]
    return sampled, points, stride


def trajectory(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append([float(row["x_m"]), float(row["y_m"]), float(row["z_m"])])
    return np.asarray(rows, dtype=np.float32)


def robust_bounds(points, a, b):
    lo = np.percentile(points[:, [a, b]], 0.05, axis=0)
    hi = np.percentile(points[:, [a, b]], 99.95, axis=0)
    span = np.maximum(hi - lo, 1.0)
    return lo - 0.03 * span, hi + 0.03 * span


def render(points, track, axes, output, label):
    width, height = 1600, 1200
    lo, hi = robust_bounds(points, *axes)
    span = hi - lo
    scale = min((width - 80) / span[0], (height - 80) / span[1])
    used = span * scale
    origin = np.array([(width - used[0]) / 2, (height - used[1]) / 2])

    xy = (points[:, list(axes)] - lo) * scale + origin
    pixel = np.rint(xy).astype(np.int32)
    pixel[:, 1] = height - 1 - pixel[:, 1]
    valid = (pixel[:, 0] >= 0) & (pixel[:, 0] < width) & (pixel[:, 1] >= 0) & (pixel[:, 1] < height)
    density = np.zeros((height, width), dtype=np.uint32)
    np.add.at(density, (pixel[valid, 1], pixel[valid, 0]), 1)
    intensity = np.log1p(density.astype(np.float32))
    if intensity.max() > 0:
        intensity *= 255.0 / intensity.max()
    image = cv2.applyColorMap(intensity.astype(np.uint8), cv2.COLORMAP_BONE)

    if len(track):
        path_xy = (track[:, list(axes)] - lo) * scale + origin
        path_px = np.rint(path_xy).astype(np.int32)
        path_px[:, 1] = height - 1 - path_px[:, 1]
        cv2.polylines(image, [path_px.reshape(-1, 1, 2)], False, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(image, tuple(path_px[0]), 8, (0, 255, 0), -1)
        cv2.circle(image, tuple(path_px[-1]), 8, (255, 0, 0), -1)

    cv2.putText(image, label + "  green=start red=trajectory blue=end", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(image, "bounds: [{:.1f},{:.1f}] to [{:.1f},{:.1f}] m".format(lo[0], lo[1], hi[0], hi[1]), (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.imwrite(output, image)
    return {"low": lo.tolist(), "high": hi.tolist(), "scale_px_per_m": float(scale)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd")
    parser.add_argument("trajectory_csv")
    parser.add_argument("output_dir")
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    points, total, stride = pcd_memmap(args.pcd)
    track = trajectory(args.trajectory_csv)
    result = {
        "pcd_points": total,
        "sample_stride": stride,
        "sampled_finite_points": int(len(points)),
        "xyz_min_sampled": points.min(axis=0).tolist(),
        "xyz_max_sampled": points.max(axis=0).tolist(),
        "top_xy": render(points, track, (0, 1), os.path.join(args.output_dir, "map_top_xy.png"), "Top XY"),
        "side_xz": render(points, track, (0, 2), os.path.join(args.output_dir, "map_side_xz.png"), "Side XZ"),
    }
    with open(os.path.join(args.output_dir, "map_render_summary.json"), "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
