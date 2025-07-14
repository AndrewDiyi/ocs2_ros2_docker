#!/bin/bash

xhost +local:docker

# 设置容器名称
CONTAINER_NAME="ocs2-ros2"

# 设置 ROS 镜像名（根据你自己的情况）
DOCKER_IMAGE="ocs2-ros2:latest"

# 检查 nvidia-container-toolkit 是否安装，决定是否添加 --gpus all
if dpkg -s nvidia-container-toolkit >/dev/null 2>&1; then
    echo "NVIDIA Container Toolkit found, enabling GPU support."
    GPU_FLAG="--gpus all"
else
    echo "NVIDIA Container Toolkit not found, running without GPU support."
    GPU_FLAG=""
fi

# 启动容器
docker run \
  --name=$CONTAINER_NAME \
  --rm \
  --interactive \
  --tty \
  --platform linux/amd64 \
  --net=host \
  $GPU_FLAG \
  --env "DISPLAY=$DISPLAY" \
  --env "QT_X11_NO_MITSHM=1" \
  --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  $DOCKER_IMAGE

