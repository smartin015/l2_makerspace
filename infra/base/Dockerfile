# This is a base file from which all other L2 making docker files should extend.
# This lets us add instrumentation and other features that interop with the space.
FROM ros:dashing

RUN apt-get update && apt-get install -y ros-dashing-demo-nodes-cpp wget && rm -rf /var/lib/apt/lists/*


# Install nvm, Node.js and node-gyp
ENV NODE_VERSION v10.16.1
RUN wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash \
    && . $HOME/.nvm/nvm.sh \
    && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

ENV PATH /bin/versions/node/$NODE_VERSION/bin:$PATH

RUN git clone https://github.com/RobotWebTools/rclnodejs.git && cd rclnodejs && git submodule update --init --recursive && npm install

COPY l2_msgs /l2_msgs
COPY templates /templates
RUN bash -c "source /opt/ros/dashing/setup.bash && cd /l2_msgs && colcon build --symlink-install"
