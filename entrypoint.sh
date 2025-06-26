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

# Start Ollama server if not running (only once)
if ! pgrep -f "ollama serve" > /dev/null; then
  echo "[Entrypoint] Starting Ollama server..."
  nohup ollama serve > /tmp/ollama.log 2>&1 &
  sleep 3
fi

# Pull the model if not present
if ! ollama list | grep -q "nomic-embed-text"; then
  echo "[Entrypoint] Pulling Ollama model 'nomic-embed-text'..."
  ollama pull nomic-embed-text
  echo "[Entrypoint] Pulling Ollama model 'gemma3:4b'..."
  ollama pull gemma3:4b
fi

exec "$@"
