#!/bin/bash
docker run \
  --net l2 --pid host -v webots:/tmp \
  -v $(pwd)/single_motor.wbt:/single_motor.wbt \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY"   -it --rm l2sim /webots_ros2/install/webots_ros2_desktop/share/webots_ros2_desktop/webots/webots --no-sandbox --stdout --stderr /single_motor.wbt
