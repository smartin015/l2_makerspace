# This is a base file from which all other L2 making docker files should extend.
# This lets us add instrumentation and other features that interop with the space.
FROM ros:foxy-ros-base
ENV ROS_WS /l2_msgs
WORKDIR $ROS_WS
COPY l2_msgs $ROS_WS/l2_msgs
COPY l2_msgs_vr $ROS_WS/l2_msgs_vr
COPY l2_msgs_sim $ROS_WS/l2_msgs_sim
COPY l2_msgs_exec $ROS_WS/l2_msgs_exec

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
COPY templates /templates
