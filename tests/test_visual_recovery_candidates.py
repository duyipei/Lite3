"""Run actual visual search callbacks with deterministic matching and ROS doubles."""
import copy
from collections import deque
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_visual_latency as fixtures


@pytest.fixture
def visual():
    harness = fixtures.VisualLatencyTests()
    harness.setUp()
    n = harness.node
    n.backend = 'orb'
    n.matcher = NS(knnMatch=lambda *args, **kwargs: [])

    class Matcher:
        delay = 0.
        translation = 0.
        def query(self, color):
            harness.now += self.delay
            return color
        def match(self, frame, *args):
            transform = np.eye(4)
            transform[0, 3] = -self.translation
            return (dict(transform=transform, inliers=30, reprojection=1.),
                    dict(matches=40, inliers=30))

    matcher = Matcher()
    n.learned_matcher = matcher
    yield harness, n, matcher
    harness.doCleanups()


def process(h, node):
    node.image_callback(h.frame(h.now))
    node.processing_callback()
    assert not h.errors


def test_raw_learned_solution_does_not_promote_reference(visual):
    h, n, _ = visual
    solutions = n.learned_solutions(h.color, [(0., n.keyframes[0])], {1})
    assert len(solutions) == 1
    assert n.learned_preferred_ids == []


def test_published_geometric_winner_promotes_reference_and_keeps_capture_stamp(visual):
    h, n, _ = visual
    process(h, n)
    assert len(h.poses) == 1
    assert n.learned_preferred_ids == [1]
    assert h.poses[0].header.stamp.sec == 10


def test_stale_learned_result_cannot_promote_reference(visual):
    h, n, matcher = visual
    matcher.delay = 5.01
    process(h, n)
    assert 'stale_query' in h.states[-1]
    assert n.learned_preferred_ids == []
    assert h.poses == []


def test_ambiguous_learned_result_cannot_promote_reference(visual):
    h, n, _ = visual
    second = copy.deepcopy(n.keyframes[0])
    second['id'] = 2
    second['map_camera'][0, 3] = 5.
    n.keyframes.append(second)
    process(h, n)
    assert 'ambiguous' in h.states[-1]
    assert n.learned_preferred_ids == []
    assert h.poses == []


def test_temporal_rejection_cannot_promote_reference(visual):
    h, n, matcher = visual
    h.params['confirmation_samples'] = 2
    n.confirmation_history = deque(maxlen=2)
    process(h, n)
    assert h.states[-1].startswith('CANDIDATE:')
    assert n.learned_preferred_ids == []
    h.now += 1.
    matcher.translation = 1.
    process(h, n)
    assert 'temporal_inconsistent' in h.states[-1]
    assert n.learned_preferred_ids == []
    assert h.poses == []


def test_failed_publish_does_not_promote_reference(visual):
    h, n, _ = visual
    def failed(_message):
        raise RuntimeError('publisher failed')
    n.pose_publisher.publish = failed
    n.image_callback(h.frame())
    n.processing_callback()
    assert h.errors and 'publisher failed' in h.errors[-1]
    assert n.learned_preferred_ids == []
    assert h.poses == []


@pytest.mark.parametrize('reason', ['position_std_0.850m', 'yaw_std_0.250rad'])
def test_outer_stability_rejection_releases_reference_without_restarting_failed_batch(visual, reason):
    h, n, _ = visual
    second = copy.deepcopy(n.keyframes[0]);second['id'] = 2
    n.keyframes.append(second)
    n.learned_preferred_ids = [1]
    n.learned_tried_ids = {1}
    n.confirmation_history.append((0., 0., 0.))
    n.initializer_state_callback(NS(data='COLLECTING:'+reason))
    assert n.learned_preferred_ids == []
    assert n.learned_tried_ids == {1}
    assert len(n.confirmation_history) == 0
    n.learned_solutions(h.color, [(0., n.keyframes[0]), (1., second)], {1, 2})
    assert n.learned_tried_ids == {1, 2}


def test_ndt_timeout_does_not_automatically_restart_initialization(visual):
    _, n, _ = visual
    n.learned_preferred_ids = [1]
    n.initializer_state_callback(NS(data='FAILED:ndt_confirmation_timeout'))
    assert n.learned_preferred_ids == [1]
    assert not n.visual_paused


def test_explicit_request_resets_active_search_once_and_only_once(visual):
    _, n, _ = visual
    n.learned_preferred_ids = [1]
    n.learned_tried_ids = {1}
    n.search_started = 10.
    n.initializer_state_callback(NS(data='COLLECTING_VISUAL_POSES'))
    assert n.learned_preferred_ids == []
    assert n.learned_tried_ids == set()
    assert n.search_started == 0.
    n.search_started = 11.
    n.initializer_state_callback(NS(data='COLLECTING_VISUAL_POSES'))
    assert n.search_started == 11.


def test_outer_rejection_cannot_expand_startup_only_strategy(visual):
    _, n, _ = visual
    n.configured_search_strategy = n.search_strategy = 'startup_only'
    n.search_mode = 'startup'
    n.initializer_state_callback(NS(data='COLLECTING:position_std_0.850m'))
    assert n.search_strategy == 'startup_only'
    assert n.search_mode == 'startup'
