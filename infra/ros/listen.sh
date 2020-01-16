#!/bin/bash
docker run --net ros_default -it ros:dashing /bin/bash -c 'ros2 topic echo /chatter'
