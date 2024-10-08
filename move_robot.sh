#!/bin/bash
if [ "$#" -ne 3 ]; then
    echo "Wrong args | Usage: $0 <topic_name> <linear_x> <angular_x>"
fi
topic_name=$1
linear_x=$2
angular_x=$3
rostopic pub /"$topic_name"/cmd_vel geometry_msgs/Twist "linear:
  x: $linear_x
  y: 0.0
  z: 0.0
angular:
  x: $angular_x
  y: 0.0
  z: 0.0"