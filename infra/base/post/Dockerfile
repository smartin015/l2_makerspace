FROM ros:eloquent-ros-core
ONBUILD ARG L2PKG
ONBUILD ARG L2NODES
ONBUILD ENV L2PKG=$L2PKG
ONBUILD ENV L2NODES=$L2NODES
ONBUILD RUN sed --in-place --expression '$isource "/node/install/setup.bash"' /ros_entrypoint.sh
ONBUILD CMD ros2 run ${L2PKG} $(echo "$L2NODES" | cut -d "." -f1)
