# docker-compose -f "docker-compose.yml" up --build ;  \
# docker run --rm -it -v ${PWD}:/arena_camera_ros2 arena_camera_ros2_ros2_arena_camera_node_dev:latest

# linux/amd64 only for now
#https://hub.docker.com/layers/osrf/ros/eloquent-desktop/images/sha256-742948bc521573ff962f5a7f084ba1562a319e547c3938603f8dff5d33d3466e?context=explore
FROM osrf/ros:humble-desktop
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
        xvfb \
        python3-pip \
        python3-colcon-common-extensions \
        git \
    && pip install \
        ultralytics==8.3.27 \
        numpy==1.26.4 \
        paddlepaddle==3.0.0b2 \
        paddleocr==2.9.1 \
        openvino==2024.1.0 \
    && rm -rf /var/lib/apt/lists/*

# ARGS might want to change ---------------------------------------------------

# ArenaSDK tar file on parent the host relative to the build context; it must contain the ArenaSDK .tar.gz file
ARG arenasdk_root_on_host=./resources/ArenaSDK/linux64 

# location of arenasdk parent folder in the container; this is where the tar file would be executed
# Note:
# if arenasdk_parent is not in root "/", the ${arenasdk_root} must be updated
ARG arenasdk_parent=/

# when the tar gets excuted in ${arenasdk_parent}, it create a folder ,arenasdk root,
# with a CONSTANT name that can not be modified which is "ArenaSDK_Linux_x64"
ARG arenasdk_root=${arenasdk_parent}/ArenaSDK_Linux_x64

# arena_api-<x.x.x.>.whl file parent folder on the host relative to the build context
ARG arena_api_root_on_host=./resources/arena_api

# location of arena_api parent folder in the container; this is where the whl file would be copied
ARG arena_api_parent=/arena_api


# install ArenaSDK ------------------------------------------------------------

# copy binraies from host into root
ADD ${arenasdk_root_on_host}/*.tar.gz ${arenasdk_parent}

# copy isntallation script from host into ArenSDK root
ADD ${arenasdk_root_on_host}/*.sh ${arenasdk_root}

# run installation script(s) to install ArenaSDK
RUN for sh_script in `ls ${arenasdk_root}/*.sh`; do sh -c $sh_script; done

# install arena_api whl -------------------------------------------------------

# copy whl file from host
ADD ${arena_api_root_on_host}/*.whl ${arena_api_parent}/

# install via pip3 all whl files in the arena_api parent dir
RUN for whl_package in `ls ${arena_api_parent}/*.whl`; do pip3 install $whl_package; done

# setup workspace -------------------------------------------------------------
ENV BASE_DIR=/arena_camera_ros2
ENV DEST_DIR=/arena_camera_ros2/ros2_ws/src/sign_recognition_node/sign_recognition_node/Distance_Sign_Recognition
ENV MODELS_PATH="$DEST_DIR/Models"

# Set PYTHONPATH in the container
ENV PYTHONPATH="$DEST_DIR:$DEST_DIR/image_processing:$DEST_DIR/utils"

# setup entrypoint
# entry point script:
#  - installs package's depens
#  - builds workspace and install it symlink way 
#  - source workspace
ADD ./arena_camera_ros_entrypoint.sh /
# make sure the entrypoint script has execute permissions
RUN chmod +x /arena_camera_ros_entrypoint.sh

# Keep the container running interactively (start bash)
ENTRYPOINT ["/bin/bash"]

# If you want the container to source your workspace before bash starts
# you can modify the entrypoint script to source the workspace

WORKDIR /arena_camera_ros2
