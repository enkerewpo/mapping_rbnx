#!/usr/bin/env bash
set -e

# Source ROS2 underlay
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace overlay
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Default RMW
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

exec "$@"
