#!/bin/bash
set -e  # Exit on error

# Export ROS environment variables
export ROS_DOMAIN_ID=17
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

ros2 run image_transport republish ffmpeg compressed \
  --ros-args \
  -r in/ffmpeg:=/camera_ffmpeg \
  -r out/compressed:=/camera_ffmpeg_repub &

exec ros2 run astroviz shelfy_dashboard_viewer
