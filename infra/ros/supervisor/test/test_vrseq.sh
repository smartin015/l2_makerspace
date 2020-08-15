#!/bin/bash
# Sends a sequence through vr_syncer
source /l2_msgs/install/setup.bash
ros2 topic pub --once /l2/vr/Sequence l2_msgs/msg/L2Sequence '{items:[{name: "pub", params: [{key: "PUBSTR", value: "this is a test message!"}]}]}'
