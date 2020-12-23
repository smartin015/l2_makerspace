ARG L2PKG=l2_ar3
ARG L2NODES="node.py moveit.py"
FROM l2moveit
FROM l2base-pre
FROM l2base-post-sim

# Copy over moveit package and all dependencies added in the ROS base installation
# TODO make ros version agnostic
COPY --from=0 /moveit/install /moveit/install
COPY --from=0 /opt/ros/foxy/lib /opt/ros/foxy/lib

COPY --from=1 /node/install /node/install

# Install shared libraries needed by moveit (?) ROS nodes
RUN apt-get update && apt-get install -y --no-install-recommends liboctomap-dev libboost-thread-dev librandom-numbers-dev libqhull-dev libfcl-dev && rm -rf /var/lib/apt/cache/*.list

RUN echo 'source "/moveit/install/setup.bash"' >> /root/.bashrc
