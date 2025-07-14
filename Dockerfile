# Base image with ROS2 Humble minimal install
FROM ubuntu:22.04 AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai \
    LANG=en_US.UTF-8 \
    ROS_DISTRO=humble \
    AMENT_PREFIX_PATH=/opt/ros/humble \
    COLCON_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib \
    PATH=/opt/ros/humble/bin:$PATH \
    PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    ROS_DOMAIN_ID=0

# Setup mirror and install core packages
RUN mv /etc/apt/sources.list /etc/apt/sources.list.bk && rm -rf /etc/apt/sources.list.d/* \
    && VERSION_CODENAME=$(grep VERSION_CODENAME /etc/os-release | cut -d'=' -f2) \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME main restricted universe multiverse" > /etc/apt/sources.list \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME-security main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME-updates main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "deb http://mirrors.sustech.edu.cn/ubuntu/ $VERSION_CODENAME-backports main restricted universe multiverse" >> /etc/apt/sources.list \
    # Install core utils, timezone and locale
    && apt-get update && apt-get install -y --no-install-recommends \
       tzdata locales curl wget gnupg2 lsb-release ca-certificates software-properties-common sudo \
    && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    # Setup ROS2 repositories
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.cloud.tencent.com/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    # Install ROS2 base
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-humble-ros-base \
       python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Development image with build tools
FROM base AS dev

ARG USERNAME=gac
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install development tools
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
    && rosdep init || echo "rosdep already initialized" \
    && rm -rf /var/lib/apt/lists/*

# Setup user account
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu; \
    fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup ROS environment
COPY --chown=$USERNAME:$USERNAME ros_entrypoint.sh /ros_entrypoint.sh
RUN echo "source /ros_entrypoint.sh" >> /home/$USERNAME/.bashrc

USER $USERNAME
RUN rosdep update

# Full desktop image with GUI tools
FROM dev AS full

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*
ENV LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib

USER $USERNAME

# Gazebo simulation support
FROM full AS gazebo

USER root
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*
  
USER $USERNAME

# Pinocchio kinematics and dynamics library
FROM gazebo AS pinocchio

USER root

# Install Pinocchio
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    && mkdir -p /etc/apt/keyrings \
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends robotpkg-py310-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# Set Pinocchio environment variables
ENV PATH="/opt/openrobots/bin:${PATH}" \
    PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}" \
    LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}" \
    PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}" \
    CMAKE_PREFIX_PATH="/opt/openrobots:${CMAKE_PREFIX_PATH}"

# Add environment variables to user's bashrc
RUN echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/$USERNAME/.bashrc \
    && echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/$USERNAME/.bashrc \
    && echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/$USERNAME/.bashrc \
    && echo 'export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH' >> /home/$USERNAME/.bashrc \
    && echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/$USERNAME/.bashrc

USER $USERNAME

# OCS2 optimal control framework
FROM pinocchio AS ocs2

USER root

# Install dependencies for OCS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnome-terminal \
    dbus-x11

WORKDIR /home/$USERNAME/ros2_ws

# Clone OCS2 repository
ARG OCS2_ROS2_COMMIT=c6dc80d9745c5f167241fdfe3d21eb11bd59e0e7
RUN git clone https://github.com/legubiao/ocs2_ros2 src/ocs2_ros2 \
    && cd src/ocs2_ros2 \
    && git checkout ${OCS2_ROS2_COMMIT} \
    && git submodule update --init --recursive \
    && chown -R $USERNAME:$USER_GID /home/$USERNAME/ros2_ws

USER $USERNAME

# Install dependencies with rosdep
RUN cd /home/$USERNAME/ros2_ws \
    && rosdep update \
    && bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build OCS2 packages
RUN cd /home/$USERNAME/ros2_ws \
    && bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-up-to ocs2_legged_robot_ros --symlink-install"
