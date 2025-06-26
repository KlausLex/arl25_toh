#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Source Catkin workspace
source /root/catkin_ws/devel/setup.bash

# Setup Conda (correct path)
source /opt/miniconda/etc/profile.d/conda.sh

# Activate environment
conda activate llm_env

# Source local workspace (if present)
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
fi

if [ -f "$ROS_WS/devel/setup.bash" ]; then
    source "$ROS_WS/devel/setup.bash"
fi

exec "$@"
