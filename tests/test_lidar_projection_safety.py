"""Offline production-path regressions; no ROS graph or robot commands."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ground', ROOT/'scripts/costmap_ground_filter.py')
G = importlib.util.module_from_spec(spec); spec.loader.exec_module(G)
spec = importlib.util.spec_from_file_location('gate_fixture', Path(__file__).with_name('test_vendor_voa_gate.py'))
F = importlib.util.module_from_spec(spec); spec.loader.exec_module(F)


def header(t=10., frame='map'):
    return NS(frame_id=frame, stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9)))


class ProjectionFreshnessTests(unittest.TestCase):
    def test_stale_future_replay_and_revocation_do_not_refresh(self):
        g = G.ProjectionFreshness()
        self.assertFalse(g.accepts(header(), 10.))
        g.update_ready(True, 10.)
        for h in [header(8.), header(11.), header(10., 'base_link')]:
            self.assertFalse(g.accepts(h, 10.))
        self.assertTrue(g.accepts(header(), 10.))
        g.commit(header())
        self.assertFalse(g.accepts(header(), 10.1))
        self.assertFalse(g.accepts(header(12.), 12.))  # stopped ready publisher
        g.update_ready(False, 12.)
        self.assertFalse(g.accepts(header(12.), 12.))
        g.update_ready(True, 12.)
        self.assertTrue(g.accepts(header(12.), 12.))
        self.assertFalse(g.accepts(header(12.), 13.6))  # compute overload


class LidarGateTests(unittest.TestCase):
    setUp = F.GateTests.setUp
    arm = F.GateTests.arm
    output = F.GateTests.output
    respond = F.GateTests.respond
    # Inherits existing VOA/command tests, with new lidar checks only enabled
    # in these cases so the baseline gate remains backwards-compatible.
    def enable_lidar(self):
        self.gate.require_lidar_costmap = True
        self.gate.counts.update({'/patrol/lidar_costmap_heartbeat': 1,
                                '/patrol/costmap_obstacles': 1})
        self.gate.timer_callback(); self.respond()

    def test_raw_lidar_heartbeat_cannot_replace_obstacle_projection(self):
        self.enable_lidar()
        self.assertFalse(self.arm().success)
        self.assertEqual(self.gate.sensor_health()[1], 'lidar_costmap_not_ready_or_stale')
        self.gate.lidar_costmap_callback(header())
        self.assertTrue(self.arm().success)
        self.gate.goal_active_callback(NS(data=True))
        self.gate.clock_time = 11.6
        self.gate.last_lidar_time = self.gate.last_odom_time = self.gate.last_ready_time = 11.6
        self.gate.timer_callback()
        self.assertEqual(self.output(), (0,0,0))
        self.assertEqual(self.gate.fault, 'lidar_costmap_not_ready_or_stale')
        self.gate.lidar_costmap_callback(header(11.6))
        self.assertFalse(self.arm().success)  # no automatic re-arm

    def test_projection_conflict_and_replays_block_motion(self):
        self.enable_lidar()
        for h in [header(8.), header(11.), header(10., 'base_link')]:
            self.gate.lidar_costmap_callback(h)
            self.assertEqual(self.gate.last_lidar_costmap_time, 0.)
        self.gate.lidar_costmap_callback(header())
        self.gate.counts['/patrol/costmap_obstacles'] = 2
        self.assertEqual(self.gate.sensor_health()[1], 'lidar_costmap_publisher_conflict')
        self.gate.clock_time = 11.
        self.gate.lidar_costmap_callback(header())
        self.assertEqual(self.gate.last_lidar_costmap_time, 10.)


class NativeImuTests(unittest.TestCase):
    def test_status_distance_invariant_to_map_origin_on_long_patrol(self):
        text = (ROOT/'vendor_patch/hdl_localization_app.hpp').read_text()
        method = text.split('// BEGIN_STATUS_RANGE')[1].split('// END_STATUS_RANGE')[0]
        code = '#include <Eigen/Geometry>\n#include <cassert>\n' + method + r'''
int main() {
 Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
 Eigen::Vector3f point(3,4,0);
 assert(WithinStatusRange(point,pose,25));
 pose(0,3)=100;pose(1,3)=-50;pose(2,3)=7;
 point+=pose.block<3,1>(0,3);
 assert(WithinStatusRange(point,pose,25));
 assert(!WithinStatusRange(point,pose,4.9));
 assert(!WithinStatusRange(Eigen::Vector3f(200,200,200),pose,25));
}
'''
        with tempfile.TemporaryDirectory() as d:
            cpp = Path(d)/'range.cpp'; cpp.write_text(code); binary = cpp.with_suffix('')
            subprocess.run(['g++','-std=c++14','-I/usr/include/eigen3',str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_actual_cpp_pair_validation(self):
        text = (ROOT/'vendor_patch/hdl_localization_app.hpp').read_text()
        method = text.split('// BEGIN_LEVELING_IMU_VALIDATION')[1].split('// END_LEVELING_IMU_VALIDATION')[0]
        code = '#include <cmath>\n#include <cassert>\n#include <limits>\n' + method + r'''
int main() {
 assert(ValidLevelingImu(.00394,0,0,0,1,0));
 assert(ValidLevelingImu(.04,0,0,0,1,0));
 assert(!ValidLevelingImu(.2005479,0,0,0,1,0));
 assert(!ValidLevelingImu(.041,0,0,0,1,0));
 assert(!ValidLevelingImu(.001,0,0,0,0,0));
 assert(!ValidLevelingImu(.001,0,0,0,1,-1));
 assert(!ValidLevelingImu(.001,0,0,0,1.1,0));
 assert(!ValidLevelingImu(.001,0,0,0,std::numeric_limits<double>::quiet_NaN(),0));
}
'''
        with tempfile.TemporaryDirectory() as d:
            cpp = Path(d)/'test.cpp'; cpp.write_text(code)
            binary = Path(d)/'test'
            subprocess.run(['g++','-std=c++14',str(cpp),'-o',str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
