#!/bin/bash
docker run --net=host -it --rm l2base:latest /bin/bash -c "source /l2_msgs/install/setup.bash && ros2 service call /vr/PushObject3D l2_msgs/PushObject3D '{object: {type: 2, name: \"test_obj\", length: 5, data: \"\"}}'"
