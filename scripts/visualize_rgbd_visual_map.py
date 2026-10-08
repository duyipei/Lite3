#!/usr/bin/env python3
"""Render the trajectory and stored RGB keyframes from a visual-map DB."""

import argparse
import os
import sqlite3

import cv2
import matplotlib.pyplot as plt
import numpy as np


def load_keyframes(database):
    connection = sqlite3.connect("file:{}?mode=ro".format(database), uri=True)
    rows = connection.execute(
        "SELECT id,map_camera,feature_count,color_jpeg FROM keyframes ORDER BY id"
    ).fetchall()
    connection.close()
    frames = []
    for keyframe_id, pose_blob, feature_count, jpeg_blob in rows:
        pose = np.frombuffer(pose_blob, dtype="<f8").reshape(4, 4).copy()
        image = cv2.imdecode(np.frombuffer(jpeg_blob, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("cannot decode keyframe {}".format(keyframe_id))
        frames.append((keyframe_id, pose, feature_count, cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))
    if not frames:
        raise RuntimeError("visual database contains no keyframes")
    return frames


def sampled_indices(count, wanted):
    return np.unique(np.linspace(0, count - 1, min(count, wanted), dtype=int))


def draw_route(axis, frames):
    positions = np.asarray([frame[1][:2, 3] for frame in frames])
    colors = np.linspace(0.0, 1.0, len(frames))
    axis.plot(positions[:, 0], positions[:, 1], color="#7f8c8d", linewidth=1.2, zorder=1)
    scatter = axis.scatter(
        positions[:, 0], positions[:, 1], c=colors, cmap="viridis", s=22,
        edgecolors="white", linewidths=0.25, zorder=2,
    )
    arrow_ids = sampled_indices(len(frames), 24)
    directions = np.asarray([frames[index][1][:2, 2] for index in arrow_ids])
    norms = np.linalg.norm(directions, axis=1)
    directions = directions / np.maximum(norms[:, None], 1e-9)
    axis.quiver(
        positions[arrow_ids, 0], positions[arrow_ids, 1],
        directions[:, 0], directions[:, 1],
        angles="xy", scale_units="xy", scale=0.75, width=0.004,
        color="#202020", alpha=0.75, zorder=3,
    )
    axis.scatter(positions[0, 0], positions[0, 1], marker="o", s=100, color="#00a651", label="start")
    axis.scatter(positions[-1, 0], positions[-1, 1], marker="X", s=100, color="#d62728", label="end")
    axis.set_title("Visual keyframe trajectory (arrows = camera view)")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linewidth=0.4, alpha=0.4)
    axis.legend(loc="best")
    return scatter


def render_overview(frames, output_path, map_name):
    figure = plt.figure(figsize=(18, 10), constrained_layout=True)
    grid = figure.add_gridspec(4, 6)
    route_axis = figure.add_subplot(grid[:, :3])
    scatter = draw_route(route_axis, frames)
    figure.colorbar(scatter, ax=route_axis, fraction=0.035, pad=0.02, label="route progress")

    indices = sampled_indices(len(frames), 12)
    for slot, index in enumerate(indices):
        axis = figure.add_subplot(grid[slot // 3, 3 + slot % 3])
        keyframe_id, pose, feature_count, image = frames[index]
        axis.imshow(image)
        axis.set_title(
            "KF {:03d}  ({:.1f}, {:.1f})  {} feat".format(
                keyframe_id, pose[0, 3], pose[1, 3], feature_count
            ),
            fontsize=9,
        )
        axis.axis("off")
    figure.suptitle("{} RGB-D visual map".format(map_name), fontsize=16)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def render_contact_sheet(frames, output_path):
    indices = sampled_indices(len(frames), 30)
    figure, axes = plt.subplots(5, 6, figsize=(18, 11), constrained_layout=True)
    for axis, index in zip(axes.flat, indices):
        keyframe_id, pose, feature_count, image = frames[index]
        axis.imshow(image)
        axis.set_title(
            "KF {:03d} | x={:.1f} y={:.1f}\n{} RGB-D features".format(
                keyframe_id, pose[0, 3], pose[1, 3], feature_count
            ),
            fontsize=8,
        )
        axis.axis("off")
    for axis in axes.flat[len(indices):]:
        axis.axis("off")
    figure.suptitle("Sampled RGB keyframes stored in the visual map", fontsize=16)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("output_dir")
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    frames = load_keyframes(args.database)
    overview = os.path.join(args.output_dir, "visual_map_overview.png")
    contact_sheet = os.path.join(args.output_dir, "visual_keyframe_contact_sheet.png")
    map_name = os.path.basename(os.path.dirname(os.path.abspath(args.database)))
    render_overview(frames, overview, map_name)
    render_contact_sheet(frames, contact_sheet)
    print("keyframes={}".format(len(frames)))
    print(overview)
    print(contact_sheet)


if __name__ == "__main__":
    main()
