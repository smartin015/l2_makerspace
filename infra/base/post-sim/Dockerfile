# FROM l2base:latest
FROM cyberbotics/webots:latest 

RUN apt-get update && apt-get -y install --no-install-recommends git curl gnupg2 lsb-release && \
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
  sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
  apt-get update && apt-get install -y --no-install-recommends ros-foxy-ros-base && \ 
  rm -rf /var/lib/apt/lists/*

COPY --from=l2base /l2_msgs/install /l2_msgs/install

ENV ROS_DISTRO=foxy
# June 2020: webots_ros2_importer is removed from the repo because ROS2 Foxy doesn't 
# yet have xacro packages and it causes an installation failure
# RUN mkdir -p /webots_ros2/src && cd /webots_ros2/src \
#    && git clone -b $ROS_DISTRO https://github.com/cyberbotics/webots_ros2.git \
#    && cd webots_ros2 && rm -r webots_ros2_importer && sed -i '/webots_ros2_importer/d' webots_ros2/package.xml \
#    && cd ../.. \
#    && apt-get update && apt-get install --no-install-recommends -y python3-pip \
#    && . /opt/ros/$ROS_DISTRO/setup.sh \
#    && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
#   && colcon build && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get -y install --no-install-recommends ros-foxy-webots-ros2 && rm -rf /var/lib/apt/lists/*

# http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
# TODO accept ARGs to select particular GPU type
#RUN \
#   apt-get update && \
#   DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends libgl1-mesa-glx libgl1-mesa-dri libnss3 libasound2 wget software-properties-common && \
#   rm -rf /var/lib/apt/lists/*

# RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add - && \
#     apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/' && \
#     apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends webots

COPY Webots-R2020a.conf /root/.config/Cyberbotics/

ONBUILD ARG L2PKG
ONBUILD ARG L2NODES
ONBUILD ENV L2PKG=$L2PKG
ONBUILD ENV L2NODES=$L2NODES

# Required for ros2 environment to also find the webots packages
# https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux&tab-language=python#environment-variables
ONBUILD ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller
ONBUILD ENV PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python38
ONBUILD RUN echo 'source "/node/install/setup.bash"\nsource "/l2_msgs/install/setup.bash"\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\nsource "/webots_ros2/install/setup.bash"' >> /root/.bashrc
ONBUILD CMD ros2 run ${L2PKG} $(echo "$L2NODES" | cut -d "." -f1)
