#!/bin/bash
# Colors
magenta=$(tput setaf 2)
normal=$(tput sgr0)
# Variables
dir=$(dirname "$0")
clear
cd "$dir/multi_ws"     # Let's enter the workspace
echo "${magenta}-------- RUNNING CATKIN_MAKE ---------${normal}"
catkin_make             # Build the project
echo "${magenta}-------- SOURCING THE WORKSPACE ---------${normal}"
source devel/setup.sh   # Source the workspace
echo "Workspace successfully sourced. Now you can run ROS nodes :)"
echo "${magenta}-----------------------------------------${normal}"