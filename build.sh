#!/bin/bash
# Colors
color=$(tput setaf 3)
normal=$(tput sgr0)
# Variables
dir=$(dirname "$0")
clear
source /opt/ros/noetic/setup.bash
echo "${color}-------- ROS IS NOW SOURCED ---------${normal}"
cd "$dir/multi_ws"     # Let's enter the workspace
echo "${color}-------- RUNNING CATKIN_MAKE ---------${normal}"
catkin_make             # Build the project
echo "${color}-------- SOURCING THE WORKSPACE ---------${normal}"
source devel/setup.sh   # Source the workspace
echo "Workspace successfully sourced. Now you can run ROS nodes :)"
cd "$dir/.."
echo "${color}-----------------------------------------${normal}"
