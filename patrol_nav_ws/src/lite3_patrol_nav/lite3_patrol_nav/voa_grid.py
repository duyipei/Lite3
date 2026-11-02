"""Validated factory GridMap decoder for map projection and diagnostics.
Unknown heights remain unknown. Coordinates include circular storage indices.
"""
import math
import numpy as np


def decode_grid(msg):
    """Decode grid_map Eigen layout and circular indices, not OccupancyGrid."""
    if msg.header.frame_id != 'robot_foot_print':
        raise ValueError('expected factory robot_foot_print elevation grid')
    if len(msg.layers) != len(msg.data) or msg.layers.count('elevation') != 1:
        raise ValueError('missing/invalid elevation layer')
    info = msg.info
    r, lx, ly = info.resolution, info.length_x, info.length_y
    pose = info.pose
    p, q = pose.position, pose.orientation
    if (not all(math.isfinite(v) for v in (r, lx, ly, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or not .01 <= r <= .2 or not 0 < lx <= 10 or not 0 < ly <= 10
            or max(abs(p.z), abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w)-1)) > 1e-6):
        raise ValueError('unsupported grid geometry/orientation')
    nx, ny = round(lx/r), round(ly/r)
    if nx <= 0 or ny <= 0 or not np.allclose([nx*r, ny*r], [lx, ly], atol=1e-6):
        raise ValueError('grid size is not an integral number of cells')
    layer = msg.data[msg.layers.index('elevation')]
    dims = layer.layout.dim
    if len(dims) != 2:
        raise ValueError('grid must have two dimensions')
    labels = tuple(d.label for d in dims)
    if labels == ('column_index', 'row_index'):
        sizes, order = (ny, nx), 'F'
    elif labels == ('row_index', 'column_index'):
        sizes, order = (nx, ny), 'C'
    else:
        raise ValueError('unknown grid matrix storage order')
    if (tuple(d.size for d in dims) != sizes
            or tuple(d.stride for d in dims) != (nx*ny, sizes[1])):
        raise ValueError('grid matrix size/stride mismatch')
    offset = layer.layout.data_offset
    if offset < 0 or len(layer.data) != offset + nx*ny:
        raise ValueError('grid matrix data size mismatch')
    elevation = np.asarray(layer.data, dtype=np.float32)[offset:].reshape((nx, ny), order=order)
    if 'observation_validity' in msg.layers:
        if msg.layers.count('observation_validity') != 1:
            raise ValueError('duplicate observation mask')
        validity = msg.data[msg.layers.index('observation_validity')]
        if ([(d.label,d.size,d.stride) for d in validity.layout.dim]
                != [(d.label,d.size,d.stride) for d in dims]
                or validity.layout.data_offset != offset or len(validity.data) != len(layer.data)):
            raise ValueError('observation mask layout mismatch')
        mask = np.asarray(validity.data,dtype=np.float32)[offset:].reshape((nx,ny),order=order)
        if (not np.isfinite(mask).all() or not ((mask==0)|(mask==1)).all()
                or not np.array_equal(mask==1,np.isfinite(elevation))):
            raise ValueError('observation mask/elevation mismatch')
    start = (msg.outer_start_index, msg.inner_start_index)
    if not (0 <= start[0] < nx and 0 <= start[1] < ny):
        raise ValueError('invalid circular grid start index')
    row, col = np.indices(elevation.shape)
    ix, iy = (row.ravel()-start[0]) % nx, (col.ravel()-start[1]) % ny
    xyz = np.column_stack((p.x + lx/2 - (ix+.5)*r,
                           p.y + ly/2 - (iy+.5)*r,
                           elevation.ravel()))
    return xyz, float(r)


def grid_obstacles(msg, minimum, include_unknown=False):
    if not math.isfinite(minimum) or minimum <= 0:
        raise ValueError('invalid obstacle threshold')
    xyz, _ = decode_grid(msg)
    finite = np.isfinite(xyz[:, 2])
    occupied = np.zeros(len(xyz), dtype=bool)
    occupied[finite] = xyz[finite, 2] > minimum
    if include_unknown:
        occupied |= ~finite
    points = xyz[occupied].copy()
    points[~np.isfinite(points[:, 2]), 2] = 0.
    return points, {'cells': len(xyz), 'obstacles': len(points),
                    'unknown_constraints': int((~finite).sum()) if include_unknown else 0,
                    'unknown': int((~finite).sum())}
