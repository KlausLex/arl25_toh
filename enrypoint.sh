#!/bin/bash
set -e

# Setup conda
export PATH="/opt/miniconda/bin:$PATH"
source /opt/miniconda/etc/profile.d/conda.sh
conda activate llm_env

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Source workspace environment if exists
if [ -f "$ROS_WS/devel/setup.bash" ]; then
    source "$ROS_WS/devel/setup.bash"
fi

exec "$@"
