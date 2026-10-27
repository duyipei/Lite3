"""ROS-independent correspondence filtering and geometric verification."""
import cv2
import numpy as np


def next_reference_batch(ranked, preferred_ids, tried_ids, limit):
    """Cover reference IDs despite live appearance ranks changing each frame.

    Keep successful references for repeated geometric confirmation. Otherwise
    try every available reference once before beginning another search pass.
    """
    preferred = [frame for frame in ranked if frame['id'] in preferred_ids]
    remaining = [frame for frame in ranked if frame['id'] not in preferred_ids]
    unseen = [frame for frame in remaining if frame['id'] not in tried_ids]
    if remaining and not unseen:
        tried_ids.clear()
        unseen = remaining
    selected = (preferred + unseen)[:limit]
    tried_ids.update(frame['id'] for frame in selected)
    return selected


def unique_ratio_matches(matcher, reference, query, ratio):
    """A current image feature can support at most one 3-D landmark."""
    if reference is None or query is None or len(reference) < 1 or len(query) < 2:
        return []
    pairs = matcher.knnMatch(reference, query, k=2)
    good = [pair[0] for pair in pairs
            if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance]
    used = set()
    unique = []
    for match in sorted(good, key=lambda item: item.distance):
        if match.trainIdx not in used:
            used.add(match.trainIdx)
            unique.append(match)
    return unique


def solve_candidate(object_points, image_points, k, min_inliers, max_error, iterations=300):
    """Refine and recheck RANSAC inliers; return diagnostics on rejection."""
    objects = np.ascontiguousarray(object_points, dtype=np.float64).reshape(-1, 3)
    pixels = np.ascontiguousarray(image_points, dtype=np.float64).reshape(-1, 2)
    detail = {'matches': len(objects), 'inliers': 0, 'reason': 'too_few_matches'}
    if len(objects) != len(pixels) or len(objects) < max(4, min_inliers):
        return None, detail
    if not (np.isfinite(objects).all() and np.isfinite(pixels).all()):
        detail['reason'] = 'invalid_points'
        return None, detail
    if (len(np.unique(pixels, axis=0)) < max(4, min_inliers)
            or np.linalg.matrix_rank(objects - objects.mean(axis=0)) < 2):
        detail['reason'] = 'degenerate_points'
        return None, detail
    try:
        ok, rvec, tvec, indices = cv2.solvePnPRansac(
            objects, pixels, k, None, iterationsCount=iterations,
            reprojectionError=max_error, confidence=0.995, flags=cv2.SOLVEPNP_AP3P,
        )
    except cv2.error:
        detail['reason'] = 'ransac_failed'
        return None, detail
    detail['inliers'] = 0 if indices is None else len(indices)
    detail['reason'] = 'insufficient_inliers'
    if not ok or detail['inliers'] < min_inliers:
        return None, detail
    indices = indices.ravel()
    # Refinement uses only geometrically consistent points.
    try:
        rvec, tvec = cv2.solvePnPRefineLM(objects[indices], pixels[indices], k, None,
                                        rvec, tvec)
    except cv2.error:
        detail['reason'] = 'refinement_failed'
        return None, detail
    rotation, _ = cv2.Rodrigues(rvec)
    if not (np.isfinite(rotation).all() and np.isfinite(tvec).all()):
        detail['reason'] = 'nonfinite_pose'
        return None, detail
    projected, _ = cv2.projectPoints(objects[indices], rvec, tvec, k, None)
    errors = np.linalg.norm(projected[:, 0, :] - pixels[indices], axis=1)
    depths = (rotation @ objects[indices].T).T[:, 2] + tvec[2, 0]
    valid = (errors <= max_error) & (depths > 0.1)
    detail['inliers'] = int(np.count_nonzero(valid))
    if detail['inliers'] < min_inliers:
        detail['reason'] = 'refinement_rejected'
        return None, detail
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec[:, 0]
    detail['reason'] = 'ok'
    return {'transform': transform, 'inliers': detail['inliers'],
            'reprojection': float(np.median(errors[valid]))}, detail
