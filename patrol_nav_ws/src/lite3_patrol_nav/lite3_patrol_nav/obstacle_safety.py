"""Goal-footprint geometry shared by endpoint clearance checks.

Runtime obstacle avoidance is handled by Nav2; this module does not read
VOA grids, constrain velocity or implement a direct stopping layer.
"""
import math

import numpy as np
import yaml


def footprint_extents(costmap):
    vertices = costmap['footprint']
    if isinstance(vertices, str):
        vertices = yaml.safe_load(vertices)
    vertices = np.asarray(vertices, dtype=float)
    padding = float(costmap.get('footprint_padding', 0.))
    if (vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3
            or not np.isfinite(vertices).all() or not math.isfinite(padding) or padding < 0):
        raise ValueError('invalid footprint')
    # A conservative centered bounding rectangle also supports asymmetric feet.
    hx, hy = np.max(np.abs(vertices), axis=0) + padding
    if not (.15 <= hx <= 1.0 and .1 <= hy <= .8):
        raise ValueError('unsupported footprint extents')
    return float(hx), float(hy)


def terminal_extents(hx, hy, yaw_tolerance, xy_tolerance, margin):
    """Cover all accepted terminal yaw/XY errors, in goal-oriented axes."""
    if not (0 <= yaw_tolerance <= math.pi and 0 <= xy_tolerance <= .5 and 0 <= margin <= .2):
        raise ValueError('invalid terminal tolerances')
    angles = [0., yaw_tolerance]
    angles += [a for a in (math.atan2(hy, hx), math.atan2(hx, hy), math.pi/2)
               if a <= yaw_tolerance]
    x = max(hx*abs(math.cos(a))+hy*abs(math.sin(a)) for a in angles)
    y = max(hy*abs(math.cos(a))+hx*abs(math.sin(a)) for a in angles)
    return x+xy_tolerance+margin, y+xy_tolerance+margin


def rectangle_hits(xy, center, yaw, half_extents, cell_radius=0.):
    d = np.asarray(xy, dtype=float).reshape(-1, 2) - center
    c, s = math.cos(yaw), math.sin(yaw)
    # Avoid waking a BLAS thread pool for thousands of tiny 2-D rotations.
    local_x = d[:, 0]*c+d[:, 1]*s
    local_y = -d[:, 0]*s+d[:, 1]*c
    return ((abs(local_x) <= half_extents[0]+cell_radius)
            & (abs(local_y) <= half_extents[1]+cell_radius))
