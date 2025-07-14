#!/bin/bash
set -e

echo "✅ 开始 ROS 2 环境测试..."

# 1. 检查 ROS 环境变量
if [[ "$ROS_DISTRO" != "humble" ]]; then
  echo "❌ ROS_DISTRO 环境变量未设置为 humble"
  exit 1
fi

# 2. 检查 ros2 命令
if ! command -v ros2 > /dev/null; then
  echo "❌ ros2 命令不可用"
  exit 1
fi

# 3. 检查 ros2 包是否存在
if ! ros2 pkg list | grep demo_nodes_cpp > /dev/null; then
  echo "❌ demo_nodes_cpp 包缺失"
  exit 1
fi

# 4. 运行 talker 测试节点（后台运行几秒即可）
echo "🔄 启动 demo_nodes_cpp talker 测试..."
ros2 run demo_nodes_cpp talker --ros-args --log-level warn &
TALKER_PID=$!
sleep 3
kill $TALKER_PID || true

# 5. 检查 rosdep
echo "🔄 运行 rosdep update 测试..."
rosdep update > /dev/null

# 6. 创建并构建一个 colcon 测试包
echo "🔧 创建并构建 colcon 测试包..."
cd ~
mkdir -p dev_ws/src && cd dev_ws/src
ros2 pkg create --build-type ament_cmake my_test_pkg > /dev/null
cd ~/dev_ws
colcon build > /dev/null

# 7. Gazebo 启动测试（非 GUI 模式）
if command -v gz > /dev/null; then
  echo "🎮 测试 headless Gazebo 启动..."
  gz sim -r > /dev/null &
  GZ_PID=$!
  sleep 3
  kill $GZ_PID || true
else
  echo "⚠️ 未检测到 gz，跳过 Gazebo 测试"
fi

echo "✅ 所有测试通过！"
