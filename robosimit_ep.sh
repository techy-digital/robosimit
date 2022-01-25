#!/usr/bin/env bash

dbus-daemon --config-file=/usr/share/dbus-1/system.conf --print-address

source /opt/ros/noetic/setup.bash

cd /home/ros/catkin_ws

source /home/ros/catkin_ws/devel/setup.bash
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=/home/ros/catkin_ws/src/srvt_ros/model/:$GAZEBO_MODEL_PATH

roscore &
sleep 3
rosnode list

nohup roslaunch srvt_moveit start_system.launch >start_system.log &
sleep 3
rosnode list

nohup rosrun srvt_moveit image_service_node.py >image_service.log &
sleep 3
rosnode list

nohup roslaunch srvt_moveit start_moveit.launch >start_moveit.log &
sleep 3
rosnode list

nohup roslaunch srvt_moveit start_rokos_task_service.launch >start_rokos_task.log &
sleep 3
rosnode list

nohup roslaunch srvt_moveit start_rokos_smach.launch >start_rokos_smach.log &
sleep 3
rosnode list

exec "$@"
