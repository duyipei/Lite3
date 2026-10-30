"""Production publication geometry preserves low returns that NDT voxels erase."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class ObstacleSamplingTests(unittest.TestCase):
 def test_low_obstacle_survives_full_pose_projection(self):
  s=(ROOT/'vendor_patch/hdl_localization_app.hpp').read_text();method=s.split('// BEGIN_OBSTACLE_GEOMETRY')[1].split('// END_OBSTACLE_GEOMETRY')[0]
  code=r'''
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <cassert>
using PointT=pcl::PointXYZI;
'''+method+r'''
int main(){
 auto raw=pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
 for(int i=0;i<100;++i){PointT p;p.x=1.01f+.0001f*i;p.y=.02f;p.z=.01f;raw->push_back(p);}
 for(float height:{.149f,.151f,.16f,.20f}){PointT p;p.x=1.02;p.y=.02;p.z=.01+height;raw->push_back(p);}
 pcl::VoxelGrid<PointT> voxel;voxel.setLeafSize(.3,.3,.3);voxel.setInputCloud(raw);
 pcl::PointCloud<PointT> centroids;voxel.filter(centroids);
 assert(centroids.size()==1 && centroids[0].z<.03); // old stream loses all four returns
 Eigen::Matrix4f P=Eigen::Matrix4f::Identity();P.block<3,3>(0,0)=Eigen::AngleAxisf(.3,Eigen::Vector3f::UnitY()).toRotationMatrix();P(2,3)=-.7;
 auto result=TransformObstacleCloud(raw,P);assert(result->size()==raw->size());
 // Exact same-frame inverse recovers obstacle heights, including 14.9 cm.
 auto recovered=TransformObstacleCloud(result,P.inverse());
 for(size_t i=0;i<raw->size();++i)assert((recovered->at(i).getVector3fMap()-raw->at(i).getVector3fMap()).norm()<1e-6);
 int retained=0;for(const auto&p:*recovered)if(p.z-.01>=.15)++retained;
 assert(retained==3);
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'cloud.cpp';c.write_text(code);exe=c.with_suffix('')
   subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3','-I/usr/include/pcl-1.10',str(c),'-o',str(exe),'-lpcl_filters','-lpcl_common'],check=True,capture_output=True)
   subprocess.run([str(exe)],check=True)
