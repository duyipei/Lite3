"""Read-only offline probes of current callbacks; no ROS/robot connections."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as N
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from test_visual_initializer_frames import InitializerTests, PoseMessage
from test_vendor_voa_gate import GateTests

results = {}
case = InitializerTests()
case.setUp()
try:
    node = case.node
    node.last_status_time = node.now
    sample = N(header=N(stamp=N(sec=1, nanosec=0)), has_converged=True,
               matching_error=.01, inlier_fraction=.001)
    node.status_callback(sample)
    results['old_low_overlap_status'] = dict(
        graph_and_monotonic_now=node.now, source_stamp=1,
        marked_good=node.scan_results.get(1000000000),
        heartbeat_count=len(node.outputs['/patrol/lidar_heartbeat']),
        last_status_arrival=node.last_status_time,
        note='Actual callback, mocked ROS only; no source-age check and only >0 inlier ratio.')
finally:
    case.doCleanups()

case = GateTests()
case.setUp()
try:
    gate = case.gate
    gate.voa.healthy = lambda now: True
    gate.counts['/patrol/cmd_vel_raw'] = 2
    results['two_raw_publishers'] = dict(health=gate.sensor_health(),
        raw_publishers=2, note='Actual gate sensor_health; all other simulated dependencies healthy.')
    gate.last_pose = (0., 0., 0.)
    gate.armed = True
    sample = PoseMessage()
    sample.header.stamp.sec = 1
    sample.pose.pose.position.x = float('nan')
    gate.odom_callback(sample)
    results['old_nonfinite_odom_at_gate'] = dict(
        source_stamp=1, now=gate.clock_time, armed=gate.armed,
        refreshed_arrival=gate.last_odom_time,
        retained_nonfinite_pose=not np.isfinite(gate.last_pose).all(),
        note='Gate alone; initializer has separate finite-pose checks, so this is defense-in-depth gap.')
finally:
    case.doCleanups()

# Evaluate the exact information-fusion algebra from correct(): a quaternion
# sign change requires S*C*S^T, including translation/quaternion cross terms.
imu = np.array([.3, 0, 0, 1, 0, 0, 0.])
odom = np.array([.1, 0, 0, 1, 0, 0, 0.])
ci = np.eye(7) * .02
co = np.eye(7) * .02
co[0, 4] = co[4, 0] = .008
sign = np.diag([1., 1, 1, -1, -1, -1, -1])
def fuse(a, ca, b, cb):
    ia, ib = np.linalg.inv(ca), np.linalg.inv(cb)
    return np.linalg.inv(ia + ib) @ (ia @ a + ib @ b)
reference = fuse(imu, ci, odom, co)
neg_mean, neg_cov = sign @ odom, sign @ co @ sign
neg_mean[3:] *= -1  # current correct() flips mean only
current = fuse(imu, ci, neg_mean, neg_cov)
results['quaternion_hemisphere_covariance'] = dict(
    formula_reference_xyz=reference[:3].tolist(),
    formula_flipped_xyz=current[:3].tolist(),
    reference_quaternion_components=reference[3:].tolist(),
    flipped_quaternion_components=current[3:].tolist(),
    fused_state_difference=float(np.linalg.norm(reference-current)),
    normalized_rotation_difference_deg=float(np.degrees(2*np.arccos(np.clip(abs(
        np.dot(reference[3:]/np.linalg.norm(reference[3:]),
               current[3:]/np.linalg.norm(current[3:]))), 0, 1)))),
    note='Numerical evaluation of source algebra, not full NDT replay. Same physical Gaussian represented with opposite quaternion sign must give equivalent fusion.')

# Consecutive rejection keeps last_correction_stamp unchanged while odom_ukf
# prediction mutates the state. This simple translation case isolates the issue.
results['repeated_cumulative_leg_delta'] = dict(
    cumulative_deltas_m=[.1, .2, .3],
    sequential_application_m=.1+.2+.3, correct_total_m=.3,
    note='Algebraic trace of current call sequence, not evidence this occurred during stationary runs.')

destination = Path(__file__).with_name('probe_results.json')
destination.write_text(json.dumps(results, indent=2, allow_nan=False))
print(json.dumps(results, indent=2, allow_nan=False))
