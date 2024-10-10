#!/bin/bash
#Init submodules
git submodule update --init --remote --recursive

BASE_DIR=$(pwd)
DEST_DIR=$(pwd)/ros2_ws/src/sign_recognition_node/sign_recognition_node/Distance_Sign_Recognition

# Export the PYTHONPATH
export PYTHONPATH="$PYTHONPATH:$DEST_DIR"
export PYTHONPATH="$PYTHONPATH:$DEST_DIR/image_processing"
export PYTHONPATH="$PYTHONPATH:$DEST_DIR/utils"

cd "$DEST_DIR/Models"

#bash download_models.sh

cd $BASE_DIR

export MODELS_PATH="$DEST_DIR/Models"

cd ros2_ws || { echo "Failed to change directory to ros2_ws"; exit 1; }

source /opt/ros/humble/setup.bash
#add condition to check that colcon builded successfully if not exit

colcon build || { echo "Colcon build failed"; exit 1; }

source install/setup.bash || { echo "Problem with sourcing install"; exit 1; }

echo $PYTHONPATH

