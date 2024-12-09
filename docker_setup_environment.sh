#!/bin/bash
#Init submodules
git submodule update --init --remote --recursive

cd "$MODELS_PATH"

bash download_models.sh

echo $BASE_DIR/ros2_ws

cd $BASE_DIR/ros2_ws

colcon build || { echo "Colcon build failed"; exit 1; }

source $BASE_DIR/ros2_ws/install/setup.bash || { echo "Problem with sourcing install"; exit 1; }

