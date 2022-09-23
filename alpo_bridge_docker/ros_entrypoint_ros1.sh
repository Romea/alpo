#!/bin/bash
set -e

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros1 environment
source "/opt/ros/$ROS1_DISTRO/setup.bash"

exec "$@"
