#!/bin/bash
# source /usr/share/gazebo/setup.sh
#source /opt/ros/dashing/setup.sh
source /l2_msgs/install/setup.sh

ENT="{name: '$1', xml: '$(cat $2)', initial_pose: {position: {x: $4, y: $5, z: $6}}}"
echo "$(ros2 service list -c) services"
echo ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' "$ENT"
ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' "$ENT"

# echo "{object: {type: 2, name: '$1', length: $(wc -m < $2), data: '$(cat $2)'}, scale: $3, pose: {position: {x: $4, y: $5, z: $6}}}" > /tmp.yml
# echo ros2 service call /l2/vr/SpawnObject3D l2_msgs/srv/SpawnObject3D "$(cat /tmp.yml)"
# ros2 service call /l2/vr/SpawnObject3D l2_msgs/srv/SpawnObject3D "$(cat /tmp.yml)"
