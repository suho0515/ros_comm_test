#!/bin/bash
set -e

# ROS 환경 설정
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source "/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source "/opt/ros/jazzy/setup.bash"
fi

# 워크스페이스 설정
if [ -f "/workspace/install/setup.bash" ]; then
  source "/workspace/install/setup.bash"
fi

exec "$@" 