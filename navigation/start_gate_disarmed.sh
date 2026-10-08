#!/usr/bin/env bash
# The gate always starts DISARMED. Arming is a separate explicit service call.
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
source "$PROJECT/navigation/patrol_env.sh"
exec ros2 run lite3_patrol_nav velocity_gate --ros-args \
  -p output_topic:=/cmd_vel_corrected \
  -p require_localization_ready:=true \
  -p freshness_timeout_s:=1.5 \
  -p command_timeout_s:=0.5 \
  -p nav_command_start_grace_s:=10.0 \
  -p nav_command_recovery_timeout_s:=8.0 \
  -p max_linear_x:=0.15 -p max_reverse_x:=0.08 \
  -p max_linear_y:=0.0 -p max_angular_z:=0.40
