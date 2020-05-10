#!/bin/bash
xhost +
#docker run   --volume=/tmp/.X11-unix:/tmp/.X11-unix   --device=/dev/dri:/dev/dri   --env="DISPLAY=$DISPLAY"   -it --rm l2sim /webots_ros2/install/webots_ros2_desktop/share/webots_ros2_desktop/webots/webots --no-sandbox


docker run   --volume=/tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY"   -it --rm l2sim /webots_ros2/install/webots_ros2_desktop/share/webots_ros2_desktop/webots/webots --no-sandbox
