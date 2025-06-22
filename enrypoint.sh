#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Source workspace environment if it exists
if [ -f "$ROS_WS/devel/setup.bash" ]; then
    source "$ROS_WS/devel/setup.bash"
fi

exec "$@"
