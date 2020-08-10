FROM l2base:latest

SHELL [ "/bin/bash", "-i", "-c" ]
# # rosbridge_suite bridge; connects, but no status handling
# RUN . /opt/ros/foxy/setup.sh && cd / && git clone -b ros2 --single-branch https://github.com/RobotWebTools/rosbridge_suite.git \
#     && cd /rosbridge_suite && colcon build \
#     && pip3 install tornado pymongo Pillow
# CMD /ros_entrypoint.sh ros2 run rosbridge_server rosbridge_websocket

# # ros2-web-bridge; no fragment handling
RUN apt-get update && apt-get install -y --no-install-recommends curl wget unzip && apt-get clean
# Install nvm, Node.js and node-gypa
ENV NODE_VERSION v12.16.3
RUN curl --silent -o- https://raw.githubusercontent.com/creationix/nvm/master/install.sh | bash
RUN . ~/.nvm/nvm.sh && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

ENV PATH /bin/versions/node/$NODE_VERSION/bin:$PATH
RUN . ~/.nvm/nvm.sh && git clone --single-branch --branch foxy-fitzroy https://github.com/RobotWebTools/rclnodejs.git && cd rclnodejs && git submodule update --init --recursive && nvm use $NODE_VERSION
#  && npm install
RUN cd / && wget https://github.com/RobotWebTools/ros2-web-bridge/archive/develop.zip \
  && unzip develop.zip && rm develop.zip \
  && mv ros2-web-bridge-develop ros2-web-bridge

RUN cd /ros2-web-bridge \
  && source ~/.bashrc && source /opt/ros/foxy/setup.bash \
  && source /l2_msgs/install/setup.bash \
  && npm install bufferutil@^4.0.1 utf-8-validate@^5.0.2 \
  && npm install
CMD ["bash", "-i", "-c", "source /opt/ros/foxy/local_setup.bash && source /l2_msgs/install/setup.bash && rm -r /ros2-web-bridge/node_modules/rclnodejs/generated cd /ros2-web-bridge && DEBUG=ros2-web-bridge:index,ros2-web-bridge:Bridge node bin/rosbridge.js --address=ws://127.0.0.1:4243"]
