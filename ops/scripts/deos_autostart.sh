#!/usr/bin/env bash
# DEOS container başlatma scripti.
# Docker içinde çalışır: /ros2_ws/ops/scripts/deos_autostart.sh
set -eo pipefail

source /opt/ros/jazzy/setup.bash
cd /ros2_ws

if [ ! -f /ros2_ws/install/setup.bash ]; then
    echo "[deos] install/ yok, colcon build yapılıyor..."
    colcon build
fi

source /ros2_ws/install/setup.bash

bash /ros2_ws/ops/scripts/startup_check.sh

python3 /ros2_ws/ops/scripts/camera_stream_server.py &

mkdir -p /ros2_ws/logs/ros_console
CONSOLE_LOG="/ros2_ws/logs/ros_console/$(date +%Y-%m-%d_%H-%M-%S).log"

exec ros2 launch vehicle_bringup main.launch.py 2>&1 | tee "$CONSOLE_LOG"
