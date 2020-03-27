# This is a base file from which all other L2 making docker files should extend.
# This lets us add instrumentation and other features that interop with the space.
FROM ros:eloquent-ros-base

# RUN apt-get update && apt-get install —no-install-recommends -y wget unzip && rm -rf /var/lib/apt/lists/*
#RUN echo "source l2_msgs/install/setup.bash" > /root/.bashrc
ENV ROS_WS /l2_msgs
WORKDIR $ROS_WS
COPY l2_msgs $ROS_WS

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd /l2_msgs && colcon build --symlink-install"

COPY templates /templates
