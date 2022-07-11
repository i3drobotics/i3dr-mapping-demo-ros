#!/bin/bash
set -e

# tail -f /dev/null

if [ -n "$HOST_ID" ]; then
    # get ip for licensing from hostid
    # more info: https://docs.gz.ro/modify-linux-hostid.html
    HOST_ID_IP_A=$(( 16#${HOST_ID:2:2} ))
    HOST_ID_IP_B=$(( 16#${HOST_ID:0:2} )) 
    HOST_ID_IP_C=$(( 16#${HOST_ID:6:2} ))
    HOST_ID_IP_D=$(( 16#${HOST_ID:4:2} ))
    HOST_ID_IP="$HOST_ID_IP_A.$HOST_ID_IP_B.$HOST_ID_IP_C.$HOST_ID_IP_D"
    # overwrite ip in hosts file
    echo "$HOST_ID_IP $(uname -n)" > /etc/hosts

    # copy license files (probably from mounted host volume)
    # for stereo library to access
    cp /root/.i3dr/lic/*.lic /root/catkin_ws/devel/lib/i3dr_stereo_camera/
fi

# source catkin workspace
source /root/catkin_ws/devel/setup.bash

# rosrun i3dr_stereo_camera init_stereo_matchers

roslaunch --wait i3dr_titania titania.launch camera_serial:=i3dr_titania_l stereo_algorithm:=2 interp:=false gain:=0 exposure:=5000 rviz:=true calibration_folder:=/root/cal