"""Compile the exact injected C++ selector with a controlled collision oracle.

Tests candidate selection, not the unchanged vendor footprint/grid algorithm.
"""
import ast
from pathlib import Path
import shutil
import subprocess

import pytest


def test_native_same_path_selection(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required for native selector test')
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root/'tools/build_vendor_voa_overlay.py').read_text())
    methods = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
               and isinstance(n.value, str) and n.value.startswith('    void CorrectForwardVelocity(')
               and '\n' in n.value]
    assert len(methods) == 1
    source = r'''
#include <memory>
#include <vector>
#include <cassert>
#include <cmath>
namespace geometry_msgs { namespace msg {
struct Twist {
 using SharedPtr=std::shared_ptr<Twist>;
 struct Vec { double x=0, y=0, z=0; } linear, angular;
}; }}
namespace grid_map_msgs { namespace msg {
struct GridMap { using SharedPtr=std::shared_ptr<GridMap>; };
}}
struct RobotVel { double x,y,yaw; RobotVel(double a,double b,double c): x(a),y(b),yaw(c) {} };
struct RobotCollisionRect {};
struct AdjustRouteParams {};
enum class RobotState { Slow, Fast };
struct Harness {
 RobotState robot_state_=RobotState::Slow;
 int calls=0;
 double allowed_x=.15, allowed_w=.4;
 RobotVel candidate{0,0,0};
 void PredictCollisionPath(grid_map_msgs::msg::GridMap::SharedPtr&, RobotVel &v,
                           std::vector<RobotCollisionRect>&) { candidate=v; }
 bool DetectCollision(grid_map_msgs::msg::GridMap::SharedPtr&, std::vector<RobotCollisionRect>&,
                      geometry_msgs::msg::Twist::SharedPtr&, AdjustRouteParams&) {
   ++calls; return candidate.x>allowed_x || std::abs(candidate.yaw)>allowed_w;
 }
''' + methods[0] + r'''
};
int main() {
 auto grid=std::make_shared<grid_map_msgs::msg::GridMap>();
 auto command=std::make_shared<geometry_msgs::msg::Twist>();
 geometry_msgs::msg::Twist out;
 RobotVel measured{0,0,0};
 command->linear.x=.12; command->angular.z=-.2;
 Harness clear;
 clear.CorrectForwardVelocity(grid, measured, command, out);
 assert(out.linear.x==.12 && out.angular.z==-.2 && clear.calls==1);
 Harness slow; slow.allowed_x=.061;
 slow.CorrectForwardVelocity(grid, measured, command, out);
 assert(out.linear.x==.06 && out.angular.z==-.1 && slow.calls==3);
 Harness blocked; blocked.allowed_x=0;
 blocked.CorrectForwardVelocity(grid, measured, command, out);
 assert(out.linear.x==0 && out.angular.z==0 && blocked.calls==4);
 Harness moving; moving.robot_state_=RobotState::Fast; moving.allowed_x=.1;
 measured=RobotVel(.14,0,0);
 moving.CorrectForwardVelocity(grid, measured, command, out);
 assert(out.linear.x==0 && out.angular.z==0 && moving.calls==1);
 command->linear.x=0; command->angular.z=.4;
 Harness rotate; rotate.allowed_w=.21;
 rotate.CorrectForwardVelocity(grid, measured, command, out);
 assert(out.linear.x==0 && out.angular.z==.2 && rotate.calls==3);
 Harness moving_clear; moving_clear.robot_state_=RobotState::Fast;
 moving_clear.CorrectForwardVelocity(grid, measured, command, out);
 assert(out.linear.x==0 && out.angular.z==.4 && moving_clear.calls==2);
}
'''
    cpp, binary = tmp_path/'selector.cpp', tmp_path/'selector'
    cpp.write_text(source)
    subprocess.run([compiler, '-std=c++14', '-O2', str(cpp), '-o', str(binary)], check=True, capture_output=True)
    subprocess.run([str(binary)], check=True, capture_output=True)
