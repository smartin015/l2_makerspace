#!/bin/bash
ENT="{name: '$1', xml: '$(cat $2)', initial_pose: {position: {x: $4, y: $5, z: $6}}}"
# echo ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' "$ENT"
docker run --rm --pid=host --net=l2  l2sim:latest bash -i -c "PYTHONUNBUFFERED=1 ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' \"$ENT\""
