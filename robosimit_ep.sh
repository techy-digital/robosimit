#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
roscore &
sleep 3
rosnode list
#rosrun gazebo_ros gzserver &
#sleep 3
#rosrun gazebo_ros gzclient &
rosrun gazebo_ros gazebo &
exec "$@"
