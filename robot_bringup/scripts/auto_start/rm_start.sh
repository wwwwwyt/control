#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/workspace/rm_ws/devel/setup.bash

source ~/workspace/rm_ws/src/control/robot_bringup/scripts/auto_start/environment/balance.sh
# if [[ $HAS_SWITCH == has ]]; then
#   export ROS_IP=192.168.100.2
# else
#   export ROS_IP=127.0.0.1
# fi
mon launch --disable-ui robot_bringup balance.launch
