"""Actual installed DWB collision critic; synthetic costmap, no ROS graph.

This checks the footprint critic, not the whole controller's trajectory search.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

CODE=r'''
#include <dwb_critics/obstacle_footprint.hpp>
#include <dwb_core/exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <cassert>
class Critic : public dwb_critics::ObstacleFootprintCritic {
public:
 void bind(nav2_costmap_2d::Costmap2D* map) {
  costmap_=map;sum_scores_=false;name_="ObstacleFootprint";scale_=1;
  for(auto xy:std::vector<std::pair<double,double>>{{.35,.25},{.35,-.25},{-.35,-.25},{-.35,.25}}) {
   geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint_spec_.push_back(p);
  }
 }
 bool legal(double y) {
  dwb_msgs::msg::Trajectory2D t;
  for(int i=0;i<=60;++i){geometry_msgs::msg::Pose2D p;p.x=.05*i;p.y=y;p.theta=0;t.poses.push_back(p);}
  try { scoreTrajectory(t);return true; }
  catch(const dwb_core::IllegalTrajectoryException&){return false;}
 }
};
int main() {
 nav2_costmap_2d::Costmap2D map(120,100,.05,-1,-2.5,0);
 Critic c;c.bind(&map);assert(c.legal(0));
 // Occupied box plus conservative inflation, straddling direct trajectory.
 for(unsigned y=40;y<=60;++y)for(unsigned x=43;x<=57;++x)map.setCost(x,y,254);
 assert(!c.legal(0));assert(c.legal(1.2));
 // Removed observation fixture: only this box disappears; no static clearing.
 for(unsigned y=40;y<=60;++y)for(unsigned x=43;x<=57;++x)map.setCost(x,y,0);
 assert(c.legal(0));
 // Unknown and out-of-map are not a traversable escape route.
 for(unsigned y=40;y<=60;++y)for(unsigned x=43;x<=57;++x)map.setCost(x,y,255);
 assert(!c.legal(0));assert(!c.legal(3.));
}
'''


class DwbCollisionTests(unittest.TestCase):
    def test_blocked_detour_removed_unknown_and_boundary(self):
        prefix=Path('/opt/ros/galactic')
        if not (prefix/'lib/libdwb_critics.so').exists(): self.skipTest('native DWB unavailable')
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'critic.cpp';source.write_text(CODE);binary=source.with_suffix('')
            command=['g++','-std=c++17',str(source),'-o',str(binary),'-I'+str(prefix/'include'),
                     '-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),
                     '-Wl,-rpath-link,'+str(prefix/'lib'),'-ldwb_critics','-ldwb_core',
                     '-lnav2_costmap_2d_core','-lrclcpp','-lrcutils']
            result=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(binary)],check=True,env=dict(os.environ,LD_LIBRARY_PATH=str(prefix/'lib')+':'+str(prefix/'lib/x86_64-linux-gnu')),timeout=10)

if __name__=='__main__':unittest.main()
