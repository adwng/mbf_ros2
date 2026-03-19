#!/bin/bash

# Source ROS 2 (change if needed)
source /opt/ros/humble/setup.bash
source ~/mbf_ws/install/setup.bash

# Resolve paths
BRINGUP_DIR=$(ros2 pkg prefix mbf_bringup)/share/mbf_bringup
DESC_DIR=$(ros2 pkg prefix mbf_description)/share/mbf_description

JOINTS_CONFIG=$BRINGUP_DIR/config/joints.yaml
GAIT_CONFIG=$BRINGUP_DIR/config/gait.yaml
LINKS_CONFIG=$BRINGUP_DIR/config/links.yaml
CONTROL_CONFIG=$BRINGUP_DIR/config/ctrl_param.yaml

XACRO_FILE=$DESC_DIR/xacro/robot.xacro

# Generate URDF from xacro
URDF=$(xacro $XACRO_FILE)

# Run node
ros2 run mbf_control mbf_control_node \
  --ros-args \
  -p urdf:="$URDF" \
  --params-file $JOINTS_CONFIG \
  --params-file $LINKS_CONFIG \
  --params-file $GAIT_CONFIG \
  --params-file $CONTROL_CONFIG