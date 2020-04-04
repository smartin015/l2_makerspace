#!/bin/bash
docker run --rm --pid=host --net=l2  l2sim:latest bash -i -c "PYTHONUNBUFFERED=1 ros2 service call /delete_entity gazebo_msgs/DeleteEntity \"{name: '$1'}\""
