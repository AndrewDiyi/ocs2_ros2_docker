FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV LANG=en_US.UTF-8

# Setup mirror
RUN mv /etc/apt/sources.list /etc/apt/sources.list.bk && rm -rf /etc/apt/sources.list.d/* \
    && VERSION_CODENAME=$(grep VERSION_CODENAME /etc/os-release | cut -d'=' -f2) \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME main restricted universe multiverse" > /etc/apt/sources.list \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME-security main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME-updates main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME-backports main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "" >> /etc/apt/sources.list

# Install core utils, timezone and locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales curl wget gnupg2 lsb-release ca-certificates software-properties-common sudo \
 && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
 && dpkg-reconfigure -f noninteractive tzdata \
 && locale-gen en_US.UTF-8 \
 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.cloud.tencent.com/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

# Set ROS env
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV ROS_DOMAIN_ID=0

###########################################
#  Develop image
###########################################
FROM base AS dev

ARG USERNAME=gac
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-humble-ament-* \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Auto-source ROS & colcon completion
COPY --chown=$USERNAME:$USERNAME ros_entrypoint.sh /ros_entrypoint.sh
RUN echo "source /ros_entrypoint.sh" >> /home/$USERNAME/.bashrc

USER $USERNAME
RUN rosdep update

###########################################
#  Full image
###########################################
FROM dev AS full

USER root
# Install the full release
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-desktop \
  && rm -rf /var/lib/apt/lists/*
ENV LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib

USER $USERNAME

###########################################
#  Full+Gazebo image
###########################################
FROM full AS gazebo

USER root
# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-ros-gz \
  && rm -rf /var/lib/apt/lists/*
  
USER $USERNAME


###################################################
#  Full+Gazebo+Pinocchio image
###################################################
FROM gazebo AS pinocchio

USER root

# Install pinocchio's dependencies
RUN apt-get update && apt-get install -qqy --no-install-recommends \
      lsb-release curl git libeigen3-dev \
    && mkdir -p /etc/apt/keyrings \
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -qqy --no-install-recommends robotpkg-py310-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# Use ENV to set environment variables. These will be available to all
# subsequent RUN commands in this stage and any stage that uses `FROM pinocchio`.
ENV PATH="/opt/openrobots/bin:${PATH}"
ENV PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}"
# This CMAKE_PREFIX_PATH is what the colcon build needs to find pinocchio.
ENV CMAKE_PREFIX_PATH="/opt/openrobots:${CMAKE_PREFIX_PATH}"

# Keep the .bashrc entries as well, so these paths are set when a user
# starts an interactive terminal in the final container.
RUN echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/$USERNAME/.bashrc && \
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/$USERNAME/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/$USERNAME/.bashrc && \
    echo 'export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH' >> /home/$USERNAME/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/$USERNAME/.bashrc


USER $USERNAME

###################################################
#  Full+Gazebo+Pinocchio+ocs2 image
###################################################

FROM pinocchio AS ocs2

USER root

# 1. 安装所有系统依赖，并进行apt-get update
#    执行完所有需要apt的操作后，再统一清理
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnome-terminal \
    dbus-x11

WORKDIR /home/$USERNAME/ros2_ws

# 2. 克隆代码库
ARG OCS2_ROS2_COMMIT=c6dc80d9745c5f167241fdfe3d21eb11bd59e0e7

#    注意这里修改了chown，确保后续非root用户可以访问
RUN git clone https://github.com/legubiao/ocs2_ros2 src/ocs2_ros2 \
    && cd src/ocs2_ros2 \
    && git checkout ${OCS2_ROS2_COMMIT} \
    && git submodule update --init --recursive \
    && chown -R $USERNAME:$USER_GID /home/$USERNAME/ros2_ws

USER $USERNAME

# 3. 现在以普通用户身份运行rosdep install
#    因为上一步没有删除apt缓存，这里可以成功安装
RUN cd /home/$USERNAME/ros2_ws \
    && rosdep update \
    && bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

USER root
RUN rm -rf /var/lib/apt/lists/*
USER $USERNAME

# 4. 编译示例
RUN cd /home/$USERNAME/ros2_ws \
    && bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-up-to ocs2_legged_robot_ros --symlink-install"
