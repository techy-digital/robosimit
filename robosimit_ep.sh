#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

cd /home/ros/catkin_ws

source /home/ros/catkin_ws/devel/setup.bash
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=/home/ros/catkin_ws/src/srvt_ros/model/:$GAZEBO_MODEL_PATH

roscore &
sleep 3
rosnode list

roslaunch srvt_moveit start_system.launch &

exec "$@"
