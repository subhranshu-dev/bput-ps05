#!/usr/bin/env bash
# Source ROS 2, the px4_msgs overlay and (if built) the project workspace.
set -e
source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi
exec "$@"
