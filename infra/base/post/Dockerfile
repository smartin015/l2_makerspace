FROM ros:foxy-ros-core
COPY --from=l2base /l2_msgs/install /l2_msgs/install
ONBUILD ARG L2PKG
ONBUILD ARG L2NODES
ONBUILD ENV L2PKG=$L2PKG
ONBUILD ENV L2NODES=$L2NODES
ONBUILD RUN sed --in-place --expression '$isource "/node/install/setup.bash"\nsource "/l2_msgs/install/setup.bash"\nsource "/opt/ros/$ROS_DISTRO/setup.bash"' /ros_entrypoint.sh
ONBUILD CMD ros2 run ${L2PKG} $(echo "$L2NODES" | cut -d "." -f1)
