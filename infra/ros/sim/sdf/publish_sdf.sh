#!/bin/bash
source /usr/share/gazebo/setup.sh
source /opt/ros/dashing/setup.bash
source /l2_msgs/install/setup.bash
ENT="{name: '$1', xml: '$(cat $2)'}"
echo "$(ros2 service list -c) services"
# echo ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' "..."
# ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' "$ENT"

echo "{object: {type: 2, name: '$1', length: $(wc -m < $2), data: '$(cat $2)'}}" > /tmp.yml
echo ros2 service call /l2/vr/PushObject3D l2_msgs/srv/PushObject3D ...
ros2 service call /vr/PushObject3D l2_msgs/srv/PushObject3D "$(cat /tmp.yml)"
