#!/bin/bash
source /l2_msgs/install/setup.bash
ros2 topic pub /l2/smartin015/active_project_id std_msgs/Int64 "{data: $1}"
