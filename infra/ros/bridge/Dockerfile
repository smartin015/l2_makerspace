FROM l2base:latest

RUN wget https://github.com/smartin015/ros2-web-bridge/archive/develop.zip \
  && unzip develop.zip && rm develop.zip \
  && mv ros2-web-bridge-develop ros2-web-bridge

RUN cd ros2-web-bridge \
  && bash -c "source /opt/ros/dashing/setup.bash \
     && source /l2_msgs/install/setup.bash \
     && npm install bufferutil@^4.0.1 utf-8-validate@^5.0.2 \
     && npm install \
     && rm -r /ros2-web-bridge/node_modules/rclnodejs/generated"

CMD ["bash", "-c", "source /opt/ros/dashing/local_setup.bash && source /l2_msgs/install/setup.bash && cd /ros2-web-bridge && DEBUG=ros2-web-bridge:index,ros2-web-bridge:Bridge node bin/rosbridge.js --address=ws://127.0.0.1:4243"]
