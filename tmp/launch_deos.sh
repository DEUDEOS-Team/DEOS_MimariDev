#!/bin/bash

# DEOS ROS2 Pipeline - Complete Launch Script
# Container: c5f464a64ab3 (deos_mimaridev-deos)
# Purpose: Launch complete autonomous vehicle system

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}DEOS ROS2 Pipeline Launcher${NC}"
echo -e "${BLUE}======================================${NC}"

# Setup environment
echo -e "${YELLOW}[1/5] Setting up ROS 2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
source install/setup.bash

# Verify packages
echo -e "${YELLOW}[2/5] Verifying packages...${NC}"
PACKAGES=$(ros2 pkg list | grep -E 'deos|vehicle|sensor|mission')
if [ -z "$PACKAGES" ]; then
    echo -e "${YELLOW}ERROR: Packages not found. Rebuild with:${NC}"
    echo "cd /ros2_ws && colcon build --packages-select deos_algorithms sensor_fusion mission_planning vehicle_controller vehicle_bringup --symlink-install"
    exit 1
fi
echo -e "${GREEN}✓ All required packages found${NC}"
echo "$PACKAGES"

# Check main launch file
echo -e "${YELLOW}[3/5] Checking launch files...${NC}"
if [ ! -f "/ros2_ws/src/vehicle_bringup/launch/main.launch.py" ]; then
    echo -e "${YELLOW}ERROR: main.launch.py not found${NC}"
    echo "Expected: /ros2_ws/src/vehicle_bringup/launch/main.launch.py"
    exit 1
fi
echo -e "${GREEN}✓ Launch file found${NC}"

# Parse arguments
echo -e "${YELLOW}[4/5] Parsing arguments...${NC}"
MISSION_FILE=""
HARDWARE_TOPIC="/hardware/motion_enable"
TIMEOUT="0.5"

while [[ $# -gt 0 ]]; do
    case $1 in
        --mission)
            MISSION_FILE="$2"
            shift 2
            ;;
        --hardware-topic)
            HARDWARE_TOPIC="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        *)
            echo -e "${YELLOW}Unknown option: $1${NC}"
            shift
            ;;
    esac
done

# Display launch configuration
echo -e "${YELLOW}[5/5] Launch Configuration:${NC}"
echo -e "  ${GREEN}✓ Environment${NC}: ROS 2 Jazzy"
echo -e "  ${GREEN}✓ Workspace${NC}: /ros2_ws"
if [ -z "$MISSION_FILE" ]; then
    echo -e "  ${YELLOW}○ Mission Mode${NC}: PASSIVE (no mission file)"
else
    echo -e "  ${GREEN}✓ Mission Mode${NC}: ACTIVE"
    echo -e "    Mission file: $MISSION_FILE"
fi
echo -e "  ${GREEN}✓ Hardware Topic${NC}: $HARDWARE_TOPIC"
echo -e "  ${GREEN}✓ Timeout${NC}: ${TIMEOUT}s"

echo ""
echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}Launching DEOS Pipeline...${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# Launch command
if [ -z "$MISSION_FILE" ]; then
    # Launch without mission
    echo -e "${GREEN}Launching in PASSIVE mode (no autonomous mission)${NC}"
    echo "Press Ctrl+C to stop"
    echo ""
    ros2 launch vehicle_bringup main.launch.py \
        hardware_motion_enable_topic:="$HARDWARE_TOPIC" \
        hardware_motion_enable_timeout_s:="$TIMEOUT"
else
    # Launch with mission
    echo -e "${GREEN}Launching in ACTIVE mode with mission: $MISSION_FILE${NC}"
    echo "Press Ctrl+C to stop"
    echo ""
    ros2 launch vehicle_bringup main.launch.py \
        mission_file:="$MISSION_FILE" \
        hardware_motion_enable_topic:="$HARDWARE_TOPIC" \
        hardware_motion_enable_timeout_s:="$TIMEOUT"
fi
