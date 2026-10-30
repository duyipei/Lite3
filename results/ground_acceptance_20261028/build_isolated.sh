#!/usr/bin/env bash
# Run on Jetson after staging this diagnostic workspace. Never installs over vendor runtime.
set -e
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
set +u
source /home/ysc/lite_cog_ros2/ndt_omp/install/setup.bash
source /home/ysc/lite_cog_ros2/fast_gicp/install/setup.bash
cd /home/ysc/lite3_patrol_project/results/ground_acceptance_20261028/ws
# The Jetson wall clock can move backwards after reboot. A clean target
# prevents future-dated old objects from masquerading as rebuilt code.
export MAKEFLAGS=-j1
nice -n 10 colcon build --packages-select hdl_localization --executor sequential --cmake-clean-first --cmake-force-configure --cmake-args -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG'
