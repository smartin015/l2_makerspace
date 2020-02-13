FROM ubuntu:bionic
#gcr.io/l2-making/base

ENV GODOT_VERSION "3.2-stable"

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
    ca-certificates \
    wget \
    unzip \
    python \
    python-openssl \
    sudo \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    libxi-dev \
    libasound-dev \
    libpulse-dev \
    libgl1-mesa-dev \
    default-jdk \ 
    libz1 \
    libncurses5 \
    libstdc++6 \
    lib32stdc++6 \
    lib32z1 \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Fetch godot installer and dependencies
RUN wget https://downloads.tuxfamily.org/godotengine/3.2/Godot_v3.2-stable_x11.64.zip \
    && wget https://downloads.tuxfamily.org/godotengine/3.2/Godot_v3.2-stable_export_templates.tpz \
    && mkdir ~/.cache \
    && mkdir -p ~/.config/godot \
    && mkdir -p ~/.local/share/godot/templates/${GODOT_VERSION}.stable \
    && unzip Godot_v3.2-stable_x11.64.zip \
    && mv Godot_v3.2-stable_x11.64.zip /usr/local/bin/godot \
    && unzip Godot_v3.2-stable_export_templates.tpz \
    && mv templates/* ~/.local/share/godot/templates/${GODOT_VERSION}.stable \
    && rm -f Godot_v3.2-stable_x11.64.zip Godot_v3.2-stable_export_templates.tpz && chmod a+x /usr/local/bin/godot

# Install platform-tools for building APK
RUN cd /tmp && wget https://dl.google.com/android/repository/platform-tools-latest-linux.zip && unzip platform-tools-latest-linux.zip && rm platform-tools-latest-linux.zip && mv platform-tools /

# Install ros2 base for C++ libraries
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
   && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

ENV ROS_DISTRO crystal
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-$ROS_DISTRO-ros-base python3-colcon-common-extensions git && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/crystal/setup.bash" > /root/.bashrc

CMD ["/Godot_v3.2-stable_x11.64", "--editor", "/projects/client/project.godot"]
