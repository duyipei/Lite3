"""Optional XFeat/LighterGlue frontend; no ROS nodes or motion commands."""
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from lite3_patrol_nav.learned_matching import depth_correspondences
from lite3_patrol_nav.visual_matching import solve_candidate


class CandidateScheduler:
    """Stable round-robin coverage with one preferred reference after a success."""
    def __init__(self):
        self.pending = []
        self.preferred = None
        self.misses = 0

    def select(self, ranked, count):
        count = max(1, count)
        by_id = {int(f['id']): f for f in ranked}
        self.pending = [i for i in self.pending if i in by_id]
        if not self.pending:
            self.pending = list(by_id)
        selected = []
        if self.preferred in by_id:
            selected.append(self.preferred)
        while self.pending and len(selected) < count:
            candidate = self.pending.pop(0)
            if candidate not in selected:
                selected.append(candidate)
        return [by_id[i] for i in selected]

    def remember(self, solutions):
        if solutions:
            self.preferred = max(solutions, key=lambda x: x['inliers'])['keyframe']
            self.misses = 0
        else:
            self.misses += 1
            if self.misses >= 3:
                self.preferred = None


class XFeatMatcher:
    def __init__(self, source, dependencies, top_k=1024):
        source, dependencies = Path(source).resolve(), Path(dependencies).resolve()
        manifest = json.loads((source/'runtime_manifest.json').read_text())
        required = {'weights/xfeat.pt', 'weights/xfeat-lighterglue.pt'}
        if not required.issubset(manifest['sha256']):
            raise RuntimeError('XFeat weight checksums missing')
        for name, expected in manifest['sha256'].items():
            if hashlib.sha256((source/name).read_bytes()).hexdigest() != expected:
                raise RuntimeError('XFeat runtime checksum mismatch: '+name)
        sys.path.insert(0, str(dependencies))
        sys.path.insert(0, str(source))
        import torch
        import kornia
        if kornia.__version__ != '0.7.2':
            raise RuntimeError('XFeat requires isolated kornia 0.7.2; loaded '+kornia.__version__)
        from modules.xfeat import XFeat
        from modules.lighterglue import LighterGlue
        torch.set_num_threads(2)
        self.torch = torch
        self.model = XFeat(weights=str(source/'weights/xfeat.pt'), top_k=top_k)
        self.model.lighterglue = LighterGlue(weights=str(source/'weights/xfeat-lighterglue.pt')).eval()
        self.weights_sha256 = hashlib.sha256((source/"weights/xfeat.pt").read_bytes()).hexdigest()
        self.references = {}
        self.scheduler = CandidateScheduler()

    def query(self, color):
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        tensor = self.torch.from_numpy(rgb.copy()).permute(2, 0, 1)[None].float()/255.
        features = self.model.detectAndCompute(tensor)[0]
        features['image_size'] = (color.shape[1], color.shape[0])
        return features

    def prepare(self, frames):
        for frame in frames:
            image = cv2.imdecode(np.frombuffer(frame['color_jpeg'], np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('invalid reference image')
            if 'xfeat_cache' in frame:
                pixels, descriptors, points, count, provenance = frame['xfeat_cache']
                if frame['xfeat_metadata']['weights_sha256'] != self.weights_sha256:
                    raise RuntimeError('cached XFeat weights differ from runtime')
                features = {
                    'keypoints': self.torch.from_numpy(np.frombuffer(pixels,'<f4').reshape(count,2).copy()).to(self.model.dev),
                    'descriptors': self.torch.from_numpy(np.frombuffer(descriptors,'<f4').reshape(count,64).copy()).to(self.model.dev),
                    'image_size': (image.shape[1],image.shape[0])}
                frame['xfeat_points'] = np.frombuffer(points,'<f4').reshape(count,3).copy()
                self.references[int(frame['id'])] = features
            else:
                self.references[int(frame['id'])] = self.query(image)

    def warmup(self):
        reference = next(iter(self.references.values()))
        self.model.match_lighterglue(reference, reference)
        if self.torch.cuda.is_available():
            self.torch.cuda.synchronize()

    def select_candidates(self, ranked, count):
        return self.scheduler.select(ranked, count)

    def remember(self, solutions):
        self.scheduler.remember(solutions)

    def match(self, frame, query, k, min_matches, min_inliers, max_error):
        reference = self.references[int(frame['id'])]
        detail = dict(raw_matches=0, matches=0, inliers=0, reason='too_few_features')
        if min(len(reference['keypoints']), len(query['keypoints'])) < 4:
            return None, detail
        p0, p1, indices = self.model.match_lighterglue(reference, query, min_conf=0.1)
        # LighterGlue already filters by confidence. Depth association applies
        # uniqueness and discontinuity checks; no old ORB XYZ is copied blindly.
        if 'xfeat_points' in frame:
            objects = frame['xfeat_points'][indices[:,0]]
            finite = np.isfinite(objects).all(axis=1)
            z = np.where(finite, objects[:,2], 0.)
            valid = finite & (z > .2) & (z < 8.)
            objects, pixels = objects[valid], p1[valid]
        else:
            objects, pixels = depth_correspondences(p0, p1, np.ones(len(p0)),
                frame['pixels'], frame['points'], k, min_confidence=0.0, max_distance=6.0)
        detail.update(raw_matches=len(p0), matches=len(objects), reason='too_few_depth_matches')
        if len(objects) < min_matches:
            return None, detail
        solution, geometry = solve_candidate(objects, pixels, k, min_inliers, max_error,
                                               iterations=1500)
        detail.update(geometry)
        if solution is not None and solution['inliers']/len(objects) < 0.35:
            detail['reason'] = 'low_inlier_ratio'
            return None, detail
        return solution, detail
