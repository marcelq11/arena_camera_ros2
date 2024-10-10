#!/bin/bash
source install/setup.bash || { echo "Problem with sourcing install"; exit 1; }

ros2 launch camera_launch camera_launch.launch.py