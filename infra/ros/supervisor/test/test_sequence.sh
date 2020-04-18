#!/bin/bash
source /l2_msgs/install/setup.bash
ros2 action send_goal /sequence l2_msgs/action/L2Sequence '{sequence: {items: [{name: "test", params: []}, {name: "test", params: []}]}}'
