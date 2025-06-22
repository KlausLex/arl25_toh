#!/bin/bash

IMAGE_NAME="arl25_toh_image:base"

echo "Building Docker image $IMAGE_NAME..."
docker build -t $IMAGE_NAME .

echo "Build complete."
