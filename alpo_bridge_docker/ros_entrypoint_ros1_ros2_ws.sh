#!/bin/bash
set -e

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros1 environment
source "/ros1_ws/devel/setup.bash"

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros2 environment
source "/opt/ros/$ROS2_DISTRO/setup.bash"
#source "/ros2_ws/install/local_setup.bash"
source "/bridge_ws/install/local_setup.bash"

exec "$@"
