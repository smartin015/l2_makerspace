ARG L2PKG=l2_ar3
ARG L2NODES="node.py moveit.py servo.py"
# FROM l2moveit
FROM l2base-pre
FROM l2base-post-sim 

# Install shared libraries needed by moveit (?) ROS nodes
RUN apt-get update && apt-get install -y --no-install-recommends liboctomap-dev libboost-thread-dev librandom-numbers-dev libqhull-dev libfcl-dev \
  ros-foxy-joint-state-publisher-gui ros-foxy-joint-state-publisher \
  python3-pip \
  && rm -rf /var/lib/apt/cache/*.list \
  && pip3 install pyzmq

# Copy over moveit package and all dependencies added in the ROS base installation
# TODO make ros version agnostic
# COPY --from=0 /moveit/install /moveit/install
# COPY --from=0 /opt/ros/foxy/lib /opt/ros/foxy/lib

COPY --from=0 /node/install /node/install

# RUN echo 'source "/node/install/setup.bash"' >> /root/.bashrc
