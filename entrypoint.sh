#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Setup conda
export PATH="/opt/miniconda/bin:$PATH"
echo 'export PATH="/opt/conda/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
source /opt/conda/etc/profile.d/conda.sh
conda activate llm_env
source devel/setup.bash

# Source workspace environment if exists
if [ -f "$ROS_WS/devel/setup.bash" ]; then
    source "$ROS_WS/devel/setup.bash"
fi

exec "$@"
