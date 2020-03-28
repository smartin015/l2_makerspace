from l2base:latest

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && apt-get update && apt-get install -y ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-pkgs \
  && rm -rf /var/lib/apt/lists/*

COPY default.world /default.world
ENV ROS_DISTRO=$ROS_DISTRO
