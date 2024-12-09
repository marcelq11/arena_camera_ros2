#!/bin/bash

# Hardcoded image name and tag
IMAGE_NAME="arena_camera_ros2:latest"
CONTAINER_NAME=${1:-"${IMAGE_NAME%%:*}-container"}
SHARED_FOLDER_PATH=${2:-"${PWD}:/arena_camera_ros2"}

# Allow GUI apps in container
xhost +local:root
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Run Docker container
docker run -it \
    --gpus all \
    --privileged \
    --network host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="$XSOCK:$XSOCK:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$SHARED_FOLDER_PATH" \
    $IMAGE_NAME
