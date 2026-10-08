#!/usr/bin/env python3
"""Offline map waypoint editor. No ROS imports, network, or motion commands."""
import argparse
import hashlib
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def load_map(path):
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text())
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only trinary occupancy maps are supported')
    resolution = float(config['resolution'])
    origin = list(map(float, config['origin']))
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3:
        raise ValueError('Invalid map geometry')
    if not all(map(math.isfinite, origin)) or abs(origin[2]) > 1e-9:
        raise ValueError('This editor requires a finite, zero-yaw map origin')
    image_path = (path.parent / config['image']).resolve()
    pixels = np.asarray(Image.open(image_path).convert('L'))
    occupancy = pixels / 255.0 if config.get('negate', 0) else 1 - pixels / 255.0
    free = occupancy < float(config['free_thresh'])
    return config, image_path, pixels, free, resolution, origin


def is_free(x, y, free, resolution, origin):
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    col = math.floor((x - origin[0]) / resolution)
    row = free.shape[0] - 1 - math.floor((y - origin[1]) / resolution)
    return 0 <= row < free.shape[0] and 0 <= col < free.shape[1] and bool(free[row, col])


def make_mission(map_path, image_path, points):
    if not points:
        raise ValueError('Select at least one waypoint')
    return {
        'mission_version': 1, 'map_id': map_path.parent.name,
        'frame_id': 'map', 'enabled': False, 'review_required': True,
        'route_source': 'offline manual selection; segments are NOT planned paths',
        'map_yaml_sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
        'map_image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
        'waypoints': [dict(id='wp_%03d' % (i + 1), x=round(float(x), 4), y=round(float(y), 4),
                           yaw=round(float(yaw), 6), pause_s=0.0)
                      for i, (x, y, yaw) in enumerate(points)]}


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=project / 'navigation/outdoor_20260921_003_area_review/patrol_map.yaml')
    parser.add_argument('--output', type=Path, help='New YAML file; existing files are never overwritten')
    args = parser.parse_args()
    map_path = args.map.resolve()
    _, image_path, pixels, free, resolution, origin = load_map(map_path)
    output = args.output or map_path.parent / ('mission_draft_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.yaml')
    # GUI imports occur only here, so geometry and export can be tested headlessly.
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button

    points, artists = [], []
    press = [None]
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.subplots_adjust(bottom=.20)
    fig.canvas.manager.set_window_title('Lite3 OFFLINE waypoint editor - NO ROBOT CONNECTION')
    h, w = pixels.shape
    ax.imshow(pixels, cmap='gray', vmin=0, vmax=255, origin='upper',
              extent=(origin[0], origin[0]+w*resolution, origin[1], origin[1]+h*resolution))
    ax.set_aspect('equal')
    ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
    ax.set_title(map_path.parent.name + '\nLeft click: add | drag: heading | right click: undo')
    ax.plot(0, 0, 'b+', markersize=12, label='Map origin (not live robot pose)')
    ax.legend(loc='upper right', fontsize=8)
    status = fig.text(.08, .035, 'OFFLINE ONLY. Dashed lines show order, NOT collision-free paths.', fontsize=10)

    def redraw(message):
        for artist in artists:
            artist.remove()
        artists.clear()
        if points:
            artists.extend(ax.plot([p[0] for p in points], [p[1] for p in points], 'o--', color='tab:orange'))
            for i, (x, y, yaw) in enumerate(points):
                artists.append(ax.text(x, y, str(i+1), color='red', fontsize=11))
                artists.append(ax.annotate('', xy=(x+.6*math.cos(yaw), y+.6*math.sin(yaw)),
                                           xytext=(x, y), arrowprops=dict(arrowstyle='->', color='red')))
        status.set_text(message)
        fig.canvas.draw_idle()

    def undo(_event=None):
        if points:
            points.pop()
        redraw('%d waypoints. Dashed lines are NOT planned paths.' % len(points))

    def on_press(event):
        press[0] = None
        if event.inaxes != ax or ax.get_navigate_mode() is not None:
            return
        if event.button == 3:
            undo()
        elif event.button == 1:
            press[0] = (event.xdata, event.ydata)

    def on_release(event):
        start, press[0] = press[0], None
        if start is None or event.inaxes != ax or event.button != 1:
            return
        x, y = start
        if not is_free(x, y, free, resolution, origin):
            redraw('Rejected: waypoint must be inside a known FREE cell.'); return
        dx, dy = event.xdata-x, event.ydata-y
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) >= .1 else 0.0
        points.append((x, y, yaw))
        redraw('Added #%d: x=%.2f y=%.2f yaw=%.1f deg. Check obstacle clearance.' % (len(points), x, y, math.degrees(yaw)))

    def return_first(_event):
        if len(points) < 2:
            redraw('Select at least two waypoints first.'); return
        points.append(points[0])
        redraw('First waypoint appended: ONE return trip, not infinite looping.')

    def save(_event):
        try:
            mission = make_mission(map_path, image_path, points)
            # Matplotlib coordinates can be numpy scalars. Serialize before
            # creating a file, so a conversion error cannot leave an empty YAML.
            payload = yaml.safe_dump(mission, allow_unicode=True, sort_keys=False)
            with output.open('x', encoding='utf-8') as stream:
                stream.write(payload)
            print('SAVED:', output, flush=True)
            redraw('SAVED (disabled, review required): ' + output.name)
        except (OSError, ValueError, yaml.YAMLError) as error:
            redraw('Save failed: ' + str(error))

    buttons = []
    for left, label, callback in [(.12, 'Undo', undo), (.38, 'Return to first', return_first), (.64, 'Save YAML', save)]:
        button = Button(fig.add_axes([left, .09, .20, .055]), label)
        button.on_clicked(callback)
        buttons.append(button)
    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('button_release_event', on_release)
    print('OFFLINE editor; no goals will be sent. Save destination:', output, flush=True)
    plt.show()


if __name__ == '__main__':
    main()
