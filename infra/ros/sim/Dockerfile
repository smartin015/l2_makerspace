ARG L2PKG=l2_sim
ARG L2NODES="node.py example_controller.py pendant_controller.py rangefinder_controller.py"

FROM l2base-pre
FROM l2base-post-sim
COPY --from=0 /node/install /node/install

ENV ROS_DISTRO=$ROS_DISTRO
