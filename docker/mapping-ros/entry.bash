#!/bin/bash
set -e

# tail -f /dev/null

# source catkin workspace
source /root/catkin_ws/devel/setup.bash

roslaunch --wait i3dr_mapping_demo mapping_demo.launch camera_type:=${I3DR_CAMERA_TYPE} camera_serial:=${I3DR_CAMERA_SERIAL}