"""Compile the actual Eigen correction and exercise 3-D convergence/bounds."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class PoseFusionTests(unittest.TestCase):
    def test_frozen_height_tilt_regression_and_motion_bounds(self):
        root = Path(__file__).resolve().parents[1]
        text = (root/'vendor_patch/hdl_localization_app.hpp').read_text()
        method = text.split('// BEGIN_BOUNDED_POSE_CORRECTION')[1].split('\n', 1)[1]
        method = method.split('// END_BOUNDED_POSE_CORRECTION')[0]
        source = r'''
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cassert>
struct Test {
''' + method + r'''
};
int main() {
  Eigen::Matrix4f expected=Eigen::Matrix4f::Identity(), measured=expected;
  expected(2,3)=.0114389f;
  expected.block<3,3>(0,0)=Eigen::AngleAxisf(-.20f,Eigen::Vector3f::UnitY()).toRotationMatrix();
  measured(0,3)=.03f; measured(1,3)=-.02f; measured(2,3)=-.5414f;
  measured.block<3,3>(0,0)=(Eigen::AngleAxisf(-.09f,Eigen::Vector3f::UnitZ())*
      Eigen::AngleAxisf(-.335f,Eigen::Vector3f::UnitY())*
      Eigen::AngleAxisf(.015f,Eigen::Vector3f::UnitX())).toRotationMatrix();
  for(int i=0;i<200;++i) {
    auto next=Test::BoundedPoseCorrection(expected,measured,.05,.01,.003490659);
    assert((next.block<3,1>(0,3)-expected.block<3,1>(0,3)).norm()<=.010001);
    assert(Eigen::Quaternionf(expected.block<3,3>(0,0)).angularDistance(
        Eigen::Quaternionf(next.block<3,3>(0,0)))<=.003491);
    assert((next.block<3,1>(0,3)-measured.block<3,1>(0,3)).norm()<=
           (expected.block<3,1>(0,3)-measured.block<3,1>(0,3)).norm()+1e-6);
    expected=next;
  }
  assert(std::abs(expected(2,3)+.5414f)<.001);
  assert(Eigen::Quaternionf(expected.block<3,3>(0,0)).angularDistance(
      Eigen::Quaternionf(measured.block<3,3>(0,0)))<.001);
  auto fixed=Test::BoundedPoseCorrection(measured,measured,.05,.01,.003490659);
  assert((fixed-measured).norm()<1e-6);
  // Quaternion shortest path across +/-pi; do not make an almost-full turn.
  expected.block<3,3>(0,0)=Eigen::AngleAxisf(3.13f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  measured.block<3,3>(0,0)=Eigen::AngleAxisf(-3.13f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  fixed=Test::BoundedPoseCorrection(expected,measured,.05,.01,.003490659);
  assert(Eigen::Quaternionf(expected.block<3,3>(0,0)).angularDistance(
      Eigen::Quaternionf(fixed.block<3,3>(0,0)))<.002);
  measured(2,3)=std::numeric_limits<float>::quiet_NaN();
  bool rejected=false;
  try { Test::BoundedPoseCorrection(expected,measured,.05,.01,.003); }
  catch(const std::runtime_error&) { rejected=true; }
  assert(rejected);
}
'''
        with tempfile.TemporaryDirectory() as d:
            cpp, binary = Path(d)/'test.cpp', Path(d)/'test'
            cpp.write_text(source)
            subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3',str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
