#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/jiaolongdu/djl/graduate_project/ros/catkin_ws/src
export ROS_HOSTNAME=localhost
