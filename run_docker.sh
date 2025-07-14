#!/bin/bash

xhost +local:

# 获取当前目录
WORKDIR=$(pwd)

# 设置容器名称
CONTAINER_NAME="ros_humble_dev"

# 设置 ROS 镜像名（根据你自己的情况）
DOCKER_IMAGE="ros2-gazebo-pinocchio-ocs2:latest"

# 启动容器
docker run \
  --name=$CONTAINER_NAME \
  --rm \
  --interactive \
  --tty \
  --platform linux/amd64 \
  --env "DISPLAY=$DISPLAY" \
  --env "QT_X11_NO_MITSHM=1" \
  --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  $DOCKER_IMAGE

