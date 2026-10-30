#!/usr/bin/env python3
"""Local gravity-level terrain classification, independent of map/NDT.

Input: metric points and actual laser origin in the SAME gravity-level frame.
A measured sensor-height interval qualifies a plane; it never shifts points or
fits installation extrinsics. Unsupported terrain is unknown, never free.
All raw returns are classified; fitting samples never replace obstacle returns.
"""
from dataclasses import dataclass
import math
import time
import numpy as np


@dataclass(frozen=True)
class GroundParameters:
    minimum_height: float = .15
    maximum_height: float = 1.20
    sensor_height_min: float = .425
    sensor_height_max: float = .585
    max_slope_deg: float = 15.
    radius: float = 4.
    seed_radius: float = 2.5
    cell: float = .12
    plane_tolerance: float = .018
    support_radius: float = .30
    min_cells: int = 45
    min_plane_fraction: float = .55


class LocalGroundModel:
    def __init__(self, parameters=None):
        self.p = parameters or GroundParameters()
        p = self.p
        if not (0 < p.minimum_height <= .15 < p.maximum_height and
                0 < p.sensor_height_min < p.sensor_height_max and
                0 < p.cell <= .15 and 0 < p.support_radius <= .5 and
                0 < p.max_slope_deg <= 20 and p.min_cells >= 20):
            raise ValueError('invalid local terrain parameters')

    @staticmethod
    def low_cells(points, size):
        """One lower quantile per XY cell; do not average obstacle heights."""
        _, inverse = np.unique(np.floor(points[:, :2]/size).astype(np.int64),
                               axis=0, return_inverse=True)
        order = np.lexsort((points[:, 2], inverse))
        count = np.bincount(inverse)
        start = np.r_[0, np.cumsum(count)[:-1]]
        # Remove isolated single-ray candidates; they cannot establish ground.
        indices = order[start + np.floor(.2*(count-1)).astype(int)]
        return points[indices[count >= 2]]

    def classify(self, xyz, laser_origin):
        start = time.monotonic()
        points = np.asarray(xyz, dtype=float)
        origin = np.asarray(laser_origin, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('invalid geometry')
        p = self.p
        finite = np.isfinite(points).all(axis=1)
        radius = np.linalg.norm(points[:, :2], axis=1)
        # Explicit sensing region. Omitted/unobserved space is NOT free space.
        roi = finite & (radius <= p.radius) & ~((abs(points[:, 0]) < .4) & (abs(points[:, 1]) < .3))
        selected = points[roi]
        seed = selected[np.linalg.norm(selected[:, :2], axis=1) <= p.seed_radius]
        seed = seed[(seed[:, 2] > origin[2]-p.sensor_height_max-.7) &
                    (seed[:, 2] < origin[2]-p.sensor_height_min+.7)]
        cells = self.low_cells(seed, p.cell) if len(seed) else seed
        result = dict(valid=False, reason='insufficient_ground_support', input=len(points),
                      roi_points=len(selected), seed_cells=len(cells), points=selected,
                      trusted=np.zeros(len(selected), dtype=bool),
                      heights=np.full(len(selected), np.nan),
                      keep=np.ones(len(selected), dtype=bool), source_indices=np.flatnonzero(roi))
        if len(cells) < p.min_cells:
            return result
        normal_limit = math.cos(math.radians(p.max_slope_deg))
        def qualified(normal, offset):
            if normal[2] < normal_limit:
                return False
            # Vertical distance, not map Z or perpendicular plane distance.
            height = (origin@normal + offset)/normal[2]
            return p.sensor_height_min <= height <= p.sensor_height_max
        rng = np.random.default_rng(491)
        triples = cells[rng.integers(0, len(cells), size=(100, 3))]
        normals = np.cross(triples[:,1]-triples[:,0], triples[:,2]-triples[:,0])
        norms = np.linalg.norm(normals, axis=1)
        good = norms > 1e-8
        triples, normals, norms = triples[good], normals[good], norms[good]
        normals /= norms[:,None]
        normals[normals[:,2]<0] *= -1
        offsets = -np.sum(normals*triples[:,0], axis=1)
        upright = normals[:,2] >= normal_limit
        normals, offsets = normals[upright], offsets[upright]
        heights = (normals@origin+offsets)/normals[:,2]
        qualified_h = (heights>=p.sensor_height_min)&(heights<=p.sensor_height_max)
        normals, offsets = normals[qualified_h], offsets[qualified_h]
        if not len(normals):
            result['reason'] = 'no_physically_supported_ground_plane';return result
        inliers = abs(cells@normals.T+offsets) <= p.plane_tolerance
        score = inliers.sum(axis=0);index=int(np.argmax(score))
        best = int(score[index]), inliers[:,index]
        if best[0] < max(p.min_cells, p.min_plane_fraction*len(cells)):
            result['reason'] = 'no_physically_supported_ground_plane';return result
        mask = best[1]
        for _ in range(3):
            ground = cells[mask];center = ground.mean(axis=0)
            _, singular, vt = np.linalg.svd(ground-center, full_matrices=False)
            normal = vt[-1]
            if normal[2] < 0:normal = -normal
            offset = -normal@center
            mask = abs(cells@normal+offset) <= p.plane_tolerance
            if not qualified(normal, offset) or mask.sum() < max(p.min_cells, p.min_plane_fraction*len(cells)):
                result['reason'] = 'plane_refinement_invalid';return result
        ground = cells[mask]
        if np.any(np.ptp(ground[:, :2], axis=0) < [.8, .6]) or singular[1] < .25:
            result['reason'] = 'ground_geometry_degenerate';return result
        # Extend support only to independently observed cells agreeing with
        # the qualified near-field plane. Never extrapolate a plane into empty
        # space or promote a raised box top to support.
        all_cells = self.low_cells(selected, p.cell)
        ground = all_cells[abs(all_cells@normal+offset) <= p.plane_tolerance]
        # Bounded spatial hash: three distinct observed ground cells within
        # support_radius. Pure numpy; no new SciPy dependency on Foxy/Jetson.
        gc = np.floor(ground[:, :2]/p.cell).astype(int)
        pc = np.floor(selected[:, :2]/p.cell).astype(int)
        lower = np.minimum(gc.min(axis=0), pc.min(axis=0))-4
        upper = np.maximum(gc.max(axis=0), pc.max(axis=0))+4
        shape = upper-lower+1
        grid = np.full(tuple(shape), -1, dtype=int)
        gi = gc-lower;grid[gi[:, 0],gi[:, 1]] = np.arange(len(ground))
        pi = pc-lower
        extent = int(np.ceil(p.support_radius/p.cell))+1
        delta=np.arange(-extent,extent+1)
        dx,dy=np.meshgrid(delta,delta,indexing='ij');dx,dy=dx.ravel(),dy.ravel()
        xx=pi[:,0,None]+dx;yy=pi[:,1,None]+dy
        inside=(xx>=0)&(xx<shape[0])&(yy>=0)&(yy<shape[1])
        ids=grid[np.clip(xx,0,shape[0]-1),np.clip(yy,0,shape[1]-1)]
        valid=inside&(ids>=0)
        near=ground[np.maximum(ids,0),:2]
        distance2=np.sum((selected[:,None,:2]-near)**2,axis=2)
        support=np.sum(valid&(distance2<=p.support_radius**2),axis=1)
        trusted = support >= 3
        heights = (selected@normal+offset)/normal[2]
        # Negative steps and unsupported returns are conservatively retained.
        # Thresholds are unchanged; there is no height averaging of obstacles.
        keep = ~trusted | (heights < -.08) | ((heights >= p.minimum_height) & (heights <= p.maximum_height))
        result.update(valid=True, reason='valid_local_plane', plane=np.r_[normal, offset].tolist(),
                      laser_height=float((origin@normal+offset)/normal[2]), trusted=trusted,
                      heights=heights, keep=keep, ground_cells=len(ground),
                      trusted_points=int(trusted.sum()), kept=int(keep.sum()),
                      fit_p95=float(np.percentile(abs(ground@normal+offset)/normal[2], 95)),
                      duration_s=time.monotonic()-start)
        return result
