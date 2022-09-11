#!/bin/bash

# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
# Full license terms provided in LICENSE.md file.

CONTAINER_NAME=$1
if [[ -z "${CONTAINER_NAME}" ]]; then
    CONTAINER_NAME=cudnn_ros_darknet
fi

# This specifies a mapping between a host directory and a directory in the
# docker container. This mapping should be changed if you wish to have access to
# a different directory
#HOST_DIR=$2
#if [[ -z "${HOST_DIR}" ]]; then
 #   HOST_DIR=`realpath ${PWD}/..`
#fi

#CONTAINER_DIR=$3
#if [[ -z "${CONTAINER_DIR}" ]]; then
#    CONTAINER_DIR=home/ubuntu/catkin_ws/src
#fi

echo "Container name     : ${CONTAINER_NAME}"
#echo "Host directory     : ${HOST_DIR}"
#echo "Container directory: ${CONTAINER_DIR}"
CUDNN_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`
if [ -z "${CUDNN_ID}" ]; then
    echo "Creating new CUDNN docker container."
    xhost +local:root
    #docker run --gpus all  -it --privileged --network=host -v ${HOST_DIR}:${CONTAINER_DIR}:rw -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env="DISPLAY" --name=${CONTAINER_NAME} cudnn_ros_darknet bash
    docker run --gpus all  -it --privileged --network=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env="DISPLAY" --name=${CONTAINER_NAME} cudnn_ros_darknet:latest bash
else
    echo "Found CUDNN docker container: ${CUDNN_ID}."
    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        xhost +local:${CUDNN_ID}
        echo "Starting and attaching to ${CONTAINER_NAME} container..."
        docker start ${CUDNN_ID}
        docker attach ${CUDNN_ID}
    else
        echo "Found running ${CONTAINER_NAME} container, attaching bash..."
        docker exec -it ${CUDNN_ID} bash
    fi
fi
