FROM l2base
ONBUILD ARG L2PKG
ONBUILD ARG L2NODES
ONBUILD WORKDIR /node
ONBUILD COPY . /node
ONBUILD RUN . /opt/ros/$ROS_DISTRO/setup.sh && /templates/setup_boilerplate.sh $L2NODES
