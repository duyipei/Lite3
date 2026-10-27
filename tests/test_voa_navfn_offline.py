"""Optional native NavFn regression using actual projected camera obstacles.

Uses the VM's installed NavFn library, no ROS graph or robot. Rasterization and
inflation here are a synthetic fixture, NOT a test of Jetson's STVL transport.
"""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('voa_fixture', Path(__file__).with_name('test_voa_costmap_bridge.py'))
FIX = importlib.util.module_from_spec(spec)
spec.loader.exec_module(FIX)

NATIVE = r'''
#include <nav2_navfn_planner/navfn.hpp>
#include <iostream>
#include <vector>
int main() {
  const int nx=120, ny=100;
  std::vector<unsigned char> cells(nx*ny);
  std::cin.read(reinterpret_cast<char*>(cells.data()), cells.size());
  if (!std::cin) return 2;
  nav2_navfn_planner::NavFn planner(nx, ny);
  planner.setCostmap(cells.data(), true, false);
  int start[2]={20,50}, goal[2]={80,50};
  planner.setStart(start); planner.setGoal(goal);
  if (!planner.calcNavFnDijkstra(true) || !planner.calcPath(nx*ny)) {
    std::cout << "NO_PATH\n"; return 0;
  }
  for(int i=0; i<planner.getPathLen(); ++i)
    std::cout << "P " << planner.getPathX()[i] << " " << planner.getPathY()[i] << "\n";
}
'''


class NativePlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        candidates = [p.parent.parent for p in Path('/opt/ros').glob('*/lib/libnav2_navfn_planner.so')]
        if not candidates or not shutil.which('g++'):
            raise unittest.SkipTest('native NavFn library/compiler unavailable')
        prefix = candidates[0]
        cls.directory = tempfile.TemporaryDirectory(prefix='voa_navfn_test_')
        cls.addClassCleanup(cls.directory.cleanup)
        source = Path(cls.directory.name)/'probe.cpp'; source.write_text(NATIVE)
        cls.binary = source.with_suffix('')
        subprocess.run(['g++', '-std=c++14', str(source), '-o', str(cls.binary),
                        '-I'+str(prefix/'include'), '-L'+str(prefix/'lib'),
                        '-Wl,-rpath,'+str(prefix/'lib'), '-Wl,-rpath-link,'+str(prefix/'lib'),
                        '-lnav2_navfn_planner'], check=True, capture_output=True)
        cls.env = dict(os.environ, LD_LIBRARY_PATH=str(prefix/'lib')+':'+os.environ.get('LD_LIBRARY_PATH', ''))

    def plan(self, costmap):
        result = subprocess.run([str(self.binary)], input=costmap.tobytes(),
                                capture_output=True, check=True, env=self.env, timeout=10)
        lines = result.stdout.decode().splitlines()
        return np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith('P ')])

    def projected_costmap(self, elevation):
        # World bounds (-1,-2.5), resolution 0.05; robot at (0,0), goal (3,0).
        msg = FIX.grid(elevation)
        xyz, _ = FIX.M.grid_obstacles(msg, .1)
        points = FIX.M.project_to_map(xyz, FIX.transform())
        y, x = np.indices((100, 120))
        distance = np.full((100, 120), np.inf)
        for px, py, _ in points:
            distance = np.minimum(distance, np.hypot((x+.5)*.05-1-px, (y+.5)*.05-2.5-py))
        # Synthetic conservative inflation to exercise the real search engine.
        cells = np.zeros((100, 120), dtype=np.uint8)
        cells[distance < .55] = 120
        cells[distance < .35] = 254
        cells[[0, -1], :] = 254; cells[:, [0, -1]] = 254
        return cells

    def test_camera_only_obstacle_changes_real_navfn_route_and_removal_restores_it(self):
        clear = np.zeros((60, 40))
        initial = self.plan(self.projected_costmap(clear))
        self.assertGreater(len(initial), 0)
        self.assertLess(np.max(np.abs(initial[:, 1]-50)), 2)
        # x ~1.5 m, y ~0; no LiDAR obstacle exists in this fixture.
        blocked = clear.copy(); blocked[13:17, 18:22] = .7
        costs = self.projected_costmap(blocked)
        detour = self.plan(costs)
        self.assertGreater(len(detour), 0)
        self.assertGreater(np.max(np.abs(detour[:, 1]-50)), 8)
        self.assertTrue(all(costs[int(y), int(x)] < 253 for x, y in detour))
        restored = self.plan(self.projected_costmap(clear))
        np.testing.assert_allclose(restored, initial)

    def test_completely_blocked_map_returns_no_path(self):
        costs = self.projected_costmap(np.zeros((60, 40)))
        costs[:, 49:52] = 254
        self.assertEqual(self.plan(costs).size, 0)


if __name__ == '__main__':
    unittest.main()
