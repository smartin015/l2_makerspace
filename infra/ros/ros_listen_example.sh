#!/bin/bash
docker run --rm --net ros_default -it l2base:latest /bin/bash -c 'ros2 topic echo /chatter'
