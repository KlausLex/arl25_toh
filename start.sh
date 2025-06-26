#!/bin/bash

IMAGE_NAME="arl25_toh_image:base"
CONTAINER_NAME="arl25_toh_container"

# Allow X server connections for GUI apps inside the container
xhost +local:docker

# Check if NVIDIA GPU is available
if command -v nvidia-smi &> /dev/null && nvidia-smi > /dev/null 2>&1; then
  echo "NVIDIA GPU detected. Running container with GPU support."

  docker run --rm -it \
    --gpus all \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device /dev/ttyUSB0:/dev/ttyUSB0 \
    --device /dev/ttyACM0:/dev/ttyACM0 \
    --device /dev/bus/usb:/dev/bus/usb \
    -v $(pwd)/catkin_ws:/home/ros/catkin_ws \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device-cgroup-rule="c 81:* rmw" \
    --device-cgroup-rule="c 189:* rmw" \
    --name $CONTAINER_NAME \
    -it \
    $IMAGE_NAME

else
  echo "No NVIDIA GPU detected. Running container without GPU support."

  docker run --rm -it \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --device /dev/ttyUSB0:/dev/ttyUSB0 \
    --device /dev/ttyACM0:/dev/ttyACM0 \
    --device /dev/bus/usb:/dev/bus/usb \
    -v $(pwd)/catkin_ws:/home/ros/catkin_ws \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/chroma_db:/persistent/chroma_db \
    --device-cgroup-rule="c 81:* rmw" \
    --device-cgroup-rule="c 189:* rmw" \
    --name $CONTAINER_NAME \
    -it \
    $IMAGE_NAME
fi