"""Optional, local-only LoFTR recovery for illumination changes.

Learned image correspondences sample depth from nearby stored RGB-D landmarks.
The sampled depth is unprojected along the actual matched reference pixel ray;
its old landmark X/Y must not be reused at a different image coordinate.
"""
import hashlib
import sys
from pathlib import Path

import cv2
import numpy as np

from lite3_patrol_nav.visual_matching import solve_candidate

WEIGHTS_SHA256 = '21f5bec5968178e8bc8b7633441836fe5de4f47d861dd2cd7dc38e271b0479ec'


class LearnedMatcher:
    def __init__(self, dependencies, weights):
        dependencies, weights = Path(dependencies), Path(weights)
        if not (dependencies/'kornia').is_dir() or not weights.is_file():
            raise RuntimeError('optional LoFTR dependencies or model weights are missing')
        if hashlib.sha256(weights.read_bytes()).hexdigest() != WEIGHTS_SHA256:
            raise RuntimeError('LoFTR model checksum mismatch')
        sys.path.insert(0, str(dependencies))
        import torch
        from kornia.feature import LoFTR
        torch.set_num_threads(2)
        self.torch = torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = LoFTR(pretrained=None)
        checkpoint = torch.load(str(weights), map_location='cpu', weights_only=True)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.eval().to(self.device)
        self.references = {}

    def tensor(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # LoFTR coarse features require dimensions divisible by eight.
        pad_h, pad_w = (-gray.shape[0]) % 8, (-gray.shape[1]) % 8
        gray = cv2.copyMakeBorder(gray, 0, pad_h, 0, pad_w, cv2.BORDER_REFLECT_101)
        return self.torch.from_numpy(gray.copy()).float()[None, None].to(self.device)/255.

    def query(self, color):
        return self.tensor(color)

    def warmup(self, color):
        """Initialize inference kernels without producing a localization pose."""
        tensor = self.tensor(color)
        with self.torch.inference_mode():
            self.model({'image0': tensor, 'image1': tensor})
        if self.device == 'cuda':
            self.torch.cuda.synchronize()

    def match(self, frame, query_tensor, k, min_matches, min_inliers, max_error):
        identifier = int(frame['id'])
        if identifier not in self.references:
            image = cv2.imdecode(np.frombuffer(frame['color_jpeg'], np.uint8), cv2.IMREAD_COLOR)
            self.references[identifier] = self.tensor(image)
        with self.torch.inference_mode():
            result = self.model({'image0': self.references[identifier], 'image1': query_tensor})
        p0 = result['keypoints0'].cpu().numpy()
        p1 = result['keypoints1'].cpu().numpy()
        confidence = result['confidence'].cpu().numpy()
        objects, pixels = depth_correspondences(p0, p1, confidence, frame['pixels'],
                                                frame['points'], k,
                                                min_confidence=0.2, max_distance=6.0)
        detail = dict(matches=len(objects), inliers=0, reason='too_few_matches')
        if len(objects) < min_matches:
            return None, detail
        solution, detail = solve_candidate(objects, pixels, k, min_inliers, max_error,
                                            iterations=1500)
        if solution is not None and solution['inliers']/len(objects) < 0.35:
            detail['reason'] = 'low_inlier_ratio'
            return None, detail
        return solution, detail


def depth_correspondences(p0, p1, confidence, reference_pixels, reference_points, k,
                          min_confidence=0.4, max_distance=4.0):
    """Use a unique nearby depth sample for each observed learned match."""
    p0, p1, confidence = np.asarray(p0), np.asarray(p1), np.asarray(confidence)
    reference_pixels = np.asarray(reference_pixels)
    reference_points = np.asarray(reference_points)
    objects, pixels, used_landmarks, used_pixels = [], [], set(), set()
    if not len(p0) or not len(reference_pixels):
        return np.empty((0, 3)), np.empty((0, 2))
    distances = ((p0[:, None, :] - reference_pixels[None, :, :])**2).sum(axis=2)
    nearest = distances.argmin(axis=1)
    for index in np.argsort(-confidence):
        if (confidence[index] < min_confidence or not np.isfinite(confidence[index])
                or not np.isfinite(p0[index]).all() or not np.isfinite(p1[index]).all()):
            continue
        landmark = int(nearest[index])
        pixel = tuple(np.round(p1[index]).astype(int))
        if (landmark in used_landmarks or pixel in used_pixels
                or distances[index, landmark] > max_distance**2):
            continue
        z = float(reference_points[landmark, 2])
        if not np.isfinite(z) or not 0.2 < z < 8.0:
            continue
        # Inverse depth is affine in image coordinates on a local plane. This
        # avoids borrowing the wrong depth along a steeply sloped road patch.
        local = distances[index] <= 12.0**2
        local_pixels = reference_pixels[local]
        local_depths = reference_points[local, 2]
        fitted = False
        if len(local_depths) >= 4 and np.all(local_depths > 0.2):
            design = np.column_stack((local_pixels-p0[index], np.ones(len(local_depths))))
            inverse = 1.0/local_depths
            coefficients, _, rank, _ = np.linalg.lstsq(design, inverse, rcond=None)
            relative_error = np.max(np.abs(design @ coefficients-inverse)/inverse)
            if rank == 3 and coefficients[2] > 0 and relative_error <= 0.03:
                prediction = 1.0/coefficients[2]
                if 0.2 < prediction < 8.0 and abs(prediction-z) <= 0.2*z:
                    z = prediction
                    fitted = True
        # Reject mixed foreground/background depth around an image edge.
        nearby = reference_points[distances[index] <= 6.0**2, 2]
        if not fitted and len(nearby) > 1 and float(np.ptp(nearby)) > max(0.15, 0.10*z):
            continue
        objects.append(((p0[index, 0]-k[0, 2])*z/k[0, 0],
                        (p0[index, 1]-k[1, 2])*z/k[1, 1], z))
        pixels.append(p1[index])
        used_landmarks.add(landmark)
        used_pixels.add(pixel)
    return np.asarray(objects, np.float64).reshape(-1, 3), np.asarray(pixels, np.float64).reshape(-1, 2)
