#!/usr/bin/env bash
set +u
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/navigation2-foxy/install/setup.bash
source /home/ysc/lite_cog_ros2/nav/install/setup.bash
source /home/ysc/lite3_patrol_project/patrol_nav_ws/install_merged/local_setup.bash
export AMENT_PREFIX_PATH=/home/ysc/lite3_patrol_project/patrol_nav_ws/install_merged:${AMENT_PREFIX_PATH}
set -u
