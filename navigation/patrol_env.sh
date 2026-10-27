#!/usr/bin/env bash
set +u
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/navigation2-foxy/install/setup.bash
source /home/ysc/lite_cog_ros2/nav/install/setup.bash
source /home/ysc/lite3_patrol_project/patrol_nav_ws/install_merged/local_setup.bash
export AMENT_PREFIX_PATH=/home/ysc/lite3_patrol_project/patrol_nav_ws/install_merged:${AMENT_PREFIX_PATH}
# Use the same middleware for project nodes, service clients and factory
# sensors. Mixed defaults on this Jetson caused missing poses/service replies.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Keep multicast participant discovery, but deliver endpoint discovery and
# sensor data by unicast. Multiple local participants otherwise receive and
# filter high-rate pointcloud multicast traffic they did not subscribe to.
# Respect an explicitly supplied Cyclone configuration.
# https://cyclonedds.io/docs/cyclonedds/latest/config/multicasting.html
if [[ -z ${CYCLONEDDS_URI:-} ]]; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
fi
set -u
