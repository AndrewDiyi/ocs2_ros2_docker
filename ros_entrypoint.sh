#!/bin/bash
set -e

# Source ROS 2 setup if available
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Source colcon argument completion if available
if [ -f "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
    source "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi

# Execute the command (can be overridden by CMD)
exec "$@"
