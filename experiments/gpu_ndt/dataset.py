#!/usr/bin/env python3
"""Prepare bounded registration-only cases from existing paired NPZ captures."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def pose_matrix(values):
    v = np.asarray(values, dtype=float)
    if v.shape != (7,) or not np.isfinite(v).all():
        raise ValueError('published pose must contain finite xyz/qxyzw')
    x, y, z, w = v[3:]
    norm = np.linalg.norm(v[3:])
    if norm < 1e-6 or abs(norm-1) > .01:
        raise ValueError('invalid recorded quaternion')
    x, y, z, w = v[3:] / norm
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = v[:3]
    return result


def write_pcd(path, xyz):
    xyz = np.asarray(xyz, dtype='<f4')
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError('cloud must be a nonempty finite Nx3 array')
    points = np.zeros((len(xyz), 4), dtype='<f4')
    points[:, :3] = xyz
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {}\nHEIGHT 1\n'
              'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA binary\n').format(len(xyz), len(xyz))
    with Path(path).open('xb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes())


def prepare(capture, map_file, output, map_leaf=.5, resolution=1.0):
    capture, map_file, output = Path(capture).resolve(), Path(map_file).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('dataset output already exists')
    if not all(math.isfinite(v) and v > 0 for v in (map_leaf, resolution)):
        raise ValueError('map leaf and NDT resolution must be finite and positive')
    if any(c.isspace() for c in str(output)):
        raise ValueError('benchmark dataset path cannot contain whitespace')
    rows = json.loads((capture/'report.json').read_text())
    if not isinstance(rows, list) or not rows or len(rows) > 4096:
        raise ValueError('capture report must contain 1..4096 frames')
    prepared = []
    previous = -1
    for index, row in enumerate(rows):
        stamp = float(row['stamp'])
        stamp_ns = int(round(stamp*1e9))
        if not math.isfinite(stamp) or stamp_ns <= previous:
            raise ValueError('recorded source stamps must increase strictly')
        previous = stamp_ns
        p = capture/('frame_%02d.npz' % index)
        with np.load(p, allow_pickle=False) as data:
            pose = pose_matrix(data['published'])
            aligned = np.asarray(data['aligned'])
            if aligned.ndim != 2 or aligned.shape[1] != 3 or not np.isfinite(aligned).all():
                raise ValueError('invalid recorded aligned cloud')
            # Recover the actual registration-frame points from the saved
            # full 3-D published transform. No guessed camera/IMU extrinsics.
            source = (aligned-pose[:3, 3]) @ pose[:3, :3]
        prepared.append((index, stamp_ns, source, pose, sha(p)))
    if not map_file.is_file():
        raise ValueError('map PCD missing')
    output.mkdir(parents=True)
    shutil.copyfile(map_file, output/'map.pcd')
    cases, frame_info = [], []
    for index, stamp_ns, source, pose, digest in prepared:
        source_path = output/('scan_%04d.pcd' % index)
        write_pcd(source_path, source)
        frame_info.append(dict(index=index, source_stamp_ns=stamp_ns, capture_sha256=digest,
                               source_points=len(source), source_pcd_sha256=sha(source_path)))
        for label, dx, yaw in [('recorded_seed', 0, 0), ('offset_seed', .20, .10)]:
            seed = pose.copy()
            seed[0, 3] += dx
            c, s = math.cos(yaw), math.sin(yaw)
            seed[:3, :3] = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ seed[:3, :3]
            cases.append('{}_{:04d}\t{}\t{}\t{}\n'.format(
                label, index, stamp_ns, source_path.name,
                '\t'.join(format(v, '.12g') for v in seed.ravel())))
    (output/'cases.tsv').write_text(''.join(cases))
    result = dict(schema_version=1, scope='registration_only', map_sha256=sha(output/'map.pcd'),
                  cases_sha256=sha(output/'cases.tsv'), map_leaf=map_leaf, resolution=resolution,
                  frames=frame_info, cases=len(cases),
                  input='inverse(recorded published pose) times recorded aligned cloud; already preprocessed',
                  seed='recorded accepted pose and explicit perturbation; neither is ground truth',
                  preprocessing='historical registration-frame points; not a replay of current raw-scan preprocessing',
                  scene='historical capture; motion/sparse/edge coverage requires separately labeled captures',
                  source_timestamps='preserved as identifiers; no ROS messages published or retimed')
    (output/'dataset.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-leaf', type=float, required=True)
    parser.add_argument('--resolution', type=float, default=1.0)
    args = parser.parse_args()
    try:
        result = prepare(args.capture, args.map, args.output, args.map_leaf, args.resolution)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, str(error)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
