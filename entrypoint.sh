#!/bin/bash
set -e

source /opt/ros/humble/setup.sh

if [ -f /app/install/setup.bash ]; then
  source /app/install/setup.bash
fi

echo "Launching wheels_node..."
exec ros2 launch wheels_encoder_reader_node wheels_encoder_reader_node.launch
