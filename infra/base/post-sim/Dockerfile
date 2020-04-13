FROM l2base:latest

RUN . /opt/ros/$ROS_DISTRO/setup.sh && mkdir -p /webots_ros2/src && cd /webots_ros2/src \
  && git clone -b $ROS_DISTRO https://github.com/cyberbotics/webots_ros2.git && cd .. \
  && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  && colcon build

# http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
# TODO accept ARGs to select particular GPU type
RUN \
  apt-get update && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri libnss3 libasound2 && \
  rm -rf /var/lib/apt/lists/*

COPY Webots-R2020a.conf /root/.config/Cyberbotics/

ONBUILD ARG L2PKG
ONBUILD ARG L2NODES
ONBUILD ENV L2PKG=$L2PKG
ONBUILD ENV L2NODES=$L2NODES
ONBUILD RUN sed --in-place --expression '$isource "/node/install/setup.bash"\nsource "/l2_msgs/install/setup.bash"\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\nsource "/webots_ros2/install/setup.bash"\nalias webots="/webots_ros2/install/webots_ros2_desktop/share/webots_ros2_desktop/webots/webots"' /ros_entrypoint.sh
ONBUILD CMD ros2 run ${L2PKG} $(echo "$L2NODES" | cut -d "." -f1)
