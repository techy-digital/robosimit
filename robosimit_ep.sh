#!/usr/bin/env bash

dbus-daemon --config-file=/usr/share/dbus-1/system.conf --print-address

source /opt/ros/noetic/setup.bash

cd /home/ros/catkin_ws

chmod +x ./src/srvt-ros/srvt_moveit/src/start_system_node.py

source /home/ros/catkin_ws/devel/setup.bash
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=/home/ros/catkin_ws/src/srvt_ros/model/:$GAZEBO_MODEL_PATH

#gazebo & >start_gazebo.log &
#sleep 10
#pkill gzclient

roscore &
sleep 10
rosnode list
#while true
#do
#    rosnode list | grep 
#    if [$testVal -ne 0]; then
#    echo "There was some error"
#fi
#    break
#done

nohup roslaunch srvt_moveit start_system.launch >start_system.log &
sleep 10
rosnode list

nohup rosrun srvt_moveit image_service_node.py >image_service.log &
sleep 10
rosnode list

nohup roslaunch srvt_moveit start_moveit.launch >start_moveit.log &
sleep 10
rosnode list

nohup roslaunch srvt_moveit start_rokos_task_service.launch >start_rokos_task.log &
sleep 10
rosnode list

nohup roslaunch srvt_moveit start_rokos_smach.launch >start_rokos_smach.log &
sleep 10
rosnode list

exec "$@"
