#!/bin/bash

echo $1
echo $2
echo $3

rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}" 
