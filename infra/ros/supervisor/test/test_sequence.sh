#!/bin/bash
source /l2_msgs/install/setup.bash
ros2 action send_goal --feedback /l2/sequence l2_msgs/action/L2Sequence '{sequence: {items:[{name: "pub", params: [{key: "topic", value: "test_topic"}, {key: "str", value: "this is a test message!"}]}]}}'
