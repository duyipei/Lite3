"""Bounded synthetic T3 pressure test; no ROS/DDS/robot and no real-time sleep.

Optional before/after benchmark:
  python3 -B tests/test_voa_tf_retry_stress.py --baseline /tmp/before.py --output /tmp/t3.json
The baseline must be a saved pre-edit source file, not a different checkout.
"""
import argparse
from collections import Counter
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest

import numpy as np

import test_voa_costmap_bridge as fixtures
from test_voa_tf_retry import Clock, ExtrapolationException, TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/t22'))
from analyze_t22 import stats_ns


SCENARIOS = {
    'normal_35ms_transport': dict(transport_ms=35),
    'jitter_35_to_98ms': dict(transport_ms=35, jitter=True),
    'late_200ms_transport': dict(transport_ms=200),
    'sustained_tf_outage': dict(transport_ms=10000, ndt_ms=10000),
    'fast_80hz_grid': dict(transport_ms=65, input_hz=80),
}


def load_baseline(path):
    spec = importlib.util.spec_from_file_location('t3_saved_baseline', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def simulate(module, duration_s=30, transport_ms=35, jitter=False, input_hz=30, ndt_ms=200):
    """TF arrival is a synthetic sample phase + transport delay, not measured DDS."""
    if not 1 <= duration_s <= 120 or not 1 <= input_hz <= 100:
        raise ValueError('bounded simulation duration/rate required')
    clock = Clock()
    epoch = clock.now_ns
    generated, failed_stamps, decoded = set(), set(), Counter()
    projection_failures, projection_attempts = set(), Counter()
    seen_at, ready_at, queue_waits = {}, {}, []
    query_count = 0
    outputs = dict(cloud=[], heartbeat=[])
    terminal, pending_counts, waits = [], [], []
    timer_wall_ns, source_ages, heartbeat_ages = [], [], []
    peak_queue = max_queries_tick = max_candidates_tick = max_publications_tick = 0
    lock_held_after_tick = 0
    process_started, wall_started = time.process_time_ns(), time.perf_counter_ns()
    real_decode, real_time = module.grid_obstacles, module.time

    def stamp_ns(header):
        return header.stamp.sec*1000000000+header.stamp.nanosec

    def decode(message, *args, **kwargs):
        decoded[stamp_ns(message.header)] += 1
        result = real_decode(message, *args, **kwargs)
        clock.advance(2)  # Explicit synthetic processing cost; not a timing measurement.
        return result

    def publisher(kind, topic, qos):
        def publish(message):
            header = message.header if topic == module.TOPIC else message
            stamp = stamp_ns(header)
            assert stamp in generated, 'unknown or restamped source'
            age = clock.now_ns-stamp
            if topic == module.TOPIC:
                outputs['cloud'].append((stamp, clock.now_ns))
                source_ages.append(age)
                queue_waits.append(ready_at[stamp]-seen_at[stamp])
                clock.advance(.25)
            else:
                outputs['heartbeat'].append((stamp, clock.now_ns))
                heartbeat_ages.append(age)
                clock.advance(.15)
        return NS(publish=publish)

    candidates_this_tick = set()
    def lookup(target, source, stamp):
        nonlocal query_count
        query_count += 1
        requested = round(stamp*1e9)
        if target == 'map': candidates_this_tick.add(requested)
        period = 100000000 if target == 'map' else 50000000
        sample = ((requested+period-1)//period)*period
        delay_ms = ndt_ms if target == 'map' else transport_ms
        if jitter and target != 'map': delay_ms += (sample//period % 10)*7
        available = sample+round(delay_ms*1000000)
        if available > clock.now_ns:
            if target == 'map': failed_stamps.add(requested)
            raise ExtrapolationException('Lookup would require extrapolation into the future')
        return NS(transform=fixtures.transform())

    class ObservedBridge(module.Bridge):
        def receive(self, message):
            seen_at.setdefault(stamp_ns(message.header), clock.now_ns)
            return super().receive(message)

        def projection_transform(self, message):
            source = stamp_ns(message.header)
            projection_attempts[source] += 1
            try:
                value = super().projection_transform(message)
            except Exception:
                projection_failures.add(source)
                raise
            ready_at[source] = clock.now_ns
            return value

        def _finish_candidate(self, candidate, state, reason):
            # Test-only observer; retains metadata, never a second GridMap cache.
            super()._finish_candidate(candidate, state, reason)
            terminal.append(dict(source_stamp_ns=candidate.source_stamp_ns, state=state,
                                 reason=reason, attempts=candidate.attempt_count))
            if state == 'PUBLISHED' and candidate.first_tf_failure_monotonic_ns is not None:
                waits.append(candidate.tf_ready_ns-candidate.first_tf_failure_monotonic_ns)

    node = NS(create_publisher=publisher, get_logger=lambda: NS(info=lambda line: None))
    heights = np.zeros((120, 80))
    heights[::7, ::7] = .3
    heights[::23, ::17] = np.nan
    template = fixtures.grid(data=heights)
    template.info.resolution = .05
    template.info.length_x, template.info.length_y = 6., 4.
    next_grid, previous_anchor = epoch, None
    step = round(1000000000/input_hz)
    source_lag_ns = 40000000
    module.grid_obstacles, module.time = decode, clock
    try:
        bridge = ObservedBridge(node, NS(lookup_transform=lookup), fixtures.Cloud, fixtures.Field,
                                NS, NS(from_msg=module.stamp_seconds), .1,
                                tf_exceptions=(TransformException,))
        for tick in range(round(duration_s/.025)):
            clock.now_ns = max(clock.now_ns, epoch+tick*25000000)
            while next_grid <= clock.now_ns:
                source_ns = next_grid-source_lag_ns
                message = copy.copy(template)
                message.header = NS(frame_id='robot_foot_print',
                                    stamp=NS(sec=source_ns//1000000000, nanosec=source_ns%1000000000))
                generated.add(source_ns)
                bridge.receive(message)
                next_grid += step
            completed_ndt = ((clock.now_ns-round(ndt_ms*1000000))//100000000)*100000000
            if completed_ndt != previous_anchor:
                registration = NS(header=NS(frame_id='map', stamp=NS(sec=completed_ndt//1000000000,
                                  nanosec=completed_ndt%1000000000)), child_frame_id='ndt_registration',
                                  pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),
                                      orientation=NS(x=0.,y=0.,z=0.,w=1.))))
                bridge.registration(registration)
                previous_anchor = completed_ndt
            peak_queue = max(peak_queue, len(bridge.pending))
            before_queries, before_publications = query_count, len(outputs['cloud'])
            candidates_this_tick.clear()
            tick_wall = time.perf_counter_ns()
            bridge.publish()
            timer_wall_ns.append(time.perf_counter_ns()-tick_wall)
            max_queries_tick = max(max_queries_tick, query_count-before_queries)
            max_candidates_tick = max(max_candidates_tick, len(candidates_this_tick))
            max_publications_tick = max(max_publications_tick, len(outputs['cloud'])-before_publications)
            lock_held_after_tick += int(bridge.lock._is_owned())
            pending_counts.append(len(bridge.pending))
        publication_stamps = [s for s, _ in outputs['cloud']]
        heartbeat_times = [at for _, at in outputs['heartbeat']]
        recovered = projection_failures & set(publication_stamps)
        reasons = Counter(row['reason'] for row in terminal)
        counts = bridge.diag_counts.copy()
        result = dict(
            simulated_duration_s=duration_s, input_hz=input_hz, source_lag_ms=40,
            grid_shape=[120, 80], tf_sampling_period_ms=50, ndt_sampling_period_ms=100,
            synthetic_transport_ms=transport_ms, synthetic_ndt_delay_ms=ndt_ms,
            synthetic_decode_cost_ms=2, synthetic_publish_cost_ms=.4,
            grid_received=len(generated), tf_lookup_calls=query_count,
            future_extrapolation_calls=counts.get('exact_tf_misses',0)+counts.get('anchor_tf_misses',0),
            exact_tf_failed_frame_count=len(failed_stamps),
            candidate_first_failure_count=len(projection_failures),
            first_failure_then_published_count=len(recovered),
            first_failure_recovery_fraction=len(recovered)/len(projection_failures) if projection_failures else None,
            candidate_projection_attempts=sum(projection_attempts.values()),
            candidate_retry_attempts=sum(max(0,n-1) for n in projection_attempts.values()),
            tf_retry_attempts=counts.get('retry_attempts'), terminal_reasons=dict(reasons),
            wait_timeout_drops=counts.get('tf_wait_timeout'), source_expired_drops=counts.get('source_expired'),
            source_budget_drops=counts.get('retry_source_budget'), retry_attempt_limit_drops=counts.get('tf_attempts_exhausted'),
            queue_max=peak_queue, queue_at_end=len(bridge.pending),
            max_tf_queries_per_tick=max_queries_tick, max_candidate_attempts_per_tick=max_candidates_tick,
            max_publications_per_tick=max_publications_tick, lock_held_after_tick=lock_held_after_tick,
            duplicate_cloud_publications=len(publication_stamps)-len(set(publication_stamps)),
            duplicate_geometry_processing=sum(max(0,n-1) for n in decoded.values()),
            max_candidate_attempts=max((x['attempts'] for x in terminal),default=None),
            published=len(publication_stamps), projected_hz=len(publication_stamps)/duration_s,
            published_fraction_of_input=len(publication_stamps)/len(generated),
            published_candidate_queue_wait=stats_ns(queue_waits),
            recovered_wait=stats_ns(waits), source_age_at_publish=stats_ns(source_ages),
            source_age_at_heartbeat=stats_ns(heartbeat_ages),
            heartbeat_interval=stats_ns([b-a for a,b in zip(heartbeat_times,heartbeat_times[1:])]),
            measured_timer_wall=stats_ns(timer_wall_ns),
            measured_cpu_ms=(time.process_time_ns()-process_started)/1e6,
            measured_wall_ms=(time.perf_counter_ns()-wall_started)/1e6,
            remaining_candidates=len(bridge.pending),
            note='Synthetic stationary geometry, TF availability and costs; no real ROS/DDS/Jetson scheduling.')
        assert result['queue_max'] <= 16
        assert result['max_candidate_attempts_per_tick'] <= 4
        assert result['max_tf_queries_per_tick'] <= 4*(1+2*12)
        assert result['max_publications_per_tick'] <= 1
        assert not result['duplicate_cloud_publications']
        assert not result['duplicate_geometry_processing']
        assert not result['lock_held_after_tick']
        assert all(a < b for a,b in zip(publication_stamps, publication_stamps[1:]))
        assert all(age <= 350000000 for age in source_ages+heartbeat_ages)
        if hasattr(bridge, 'tf_retry_max_wait_ns'):
            assert all(wait <= bridge.tf_retry_max_wait_ns for wait in waits)
            assert (result['max_candidate_attempts'] or 0) <= bridge.tf_retry_max_attempts
            assert result['first_failure_then_published_count'] == bridge.diag_counts['retry_recovered']
            assert result['candidate_retry_attempts'] == bridge.diag_counts['retry_attempts']
        return result
    finally:
        module.grid_obstacles, module.time = real_decode, real_time


class PressureTests(unittest.TestCase):
    def test_jittered_stream_remains_bounded_without_duplicate_or_expired_publications(self):
        result = simulate(fixtures.M, duration_s=3, **SCENARIOS['jitter_35_to_98ms'])
        self.assertGreater(result['first_failure_then_published_count'], 10)
        self.assertGreater(result['projected_hz'], 5)
        self.assertLessEqual(result['queue_max'], 16)

    def test_fast_input_remains_bounded(self):
        result = simulate(fixtures.M, duration_s=3, **SCENARIOS['fast_80hz_grid'])
        self.assertGreater(result['published'], 10)
        self.assertLessEqual(result['max_candidate_attempts_per_tick'], 4)

    def test_continuous_tf_outage_produces_no_heartbeat_or_republished_data(self):
        result = simulate(fixtures.M, duration_s=3, **SCENARIOS['sustained_tf_outage'])
        self.assertEqual(result['published'], 0)
        self.assertGreater(result['wait_timeout_drops'], 0)
        self.assertLessEqual(result['queue_max'], 16)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=30)
    args = parser.parse_args()
    baseline = load_baseline(args.baseline)
    results = {name:dict(before=simulate(baseline,duration_s=args.duration,**options),
                        after=simulate(fixtures.M,duration_s=args.duration,**options))
               for name,options in SCENARIOS.items()}
    report = dict(baseline_source=str(args.baseline),
                  baseline_sha256=hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
                  current_sha256=hashlib.sha256(Path(fixtures.M.__file__).read_bytes()).hexdigest(),
                  scenarios=results,
                  limitations=['Not real ROS/DDS/Jetson performance.',
                      'The saved baseline already retried across timers; do not attribute all recoveries to T3.',
                      'Processing/publish costs and TF availability are explicitly synthetic.',
                      'Remaining candidates at the simulation end are pending, not counted as dropped.',
                      'Measured CPU includes the test harness, grid fixtures and instrumentation.'])
    with args.output.open('x') as handle:json.dump(report,handle,ensure_ascii=False,indent=2)
    for name,pair in results.items():
        print(name,json.dumps({side:{k:r[k] for k in ['published','projected_hz','first_failure_recovery_fraction',
            'future_extrapolation_calls','wait_timeout_drops','source_budget_drops','queue_max','measured_cpu_ms']}
            for side,r in pair.items()}))
    print('REPORT',args.output)


if __name__ == '__main__':
    if '--baseline' in sys.argv:
        main()
    else:
        unittest.main()
