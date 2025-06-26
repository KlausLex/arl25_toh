#!/bin/bash
set -e

# Start Ollama (if not running externally)
# Optional: `ollama serve &` if you're managing it yourself

# Ensure model is pulled
if ! ollama list | grep -q "nomic-embed-text"; then
  echo "Pulling Ollama model 'nomic-embed-text'..."
  ollama pull nomic-embed-text
fi

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
