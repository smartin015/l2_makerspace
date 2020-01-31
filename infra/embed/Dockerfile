FROM ubuntu:19.04
MAINTAINER Scott Martin <smartin015@gmail.com>

RUN apt-get update && apt-get install -y --fix-missing curl git python python-pip bash tmux build-essential cmake libsfml-dev vim && rm -rf /var/lib/apt/lists/*
RUN pip install -U platformio paho-mqtt

# RUN curl https://codeload.github.com/google/flatbuffers/tar.gz/v1.11.0 -o /tmp/flatbuffers.tar.gz && cd /tmp && tar xvf flatbuffers.tar.gz && mkdir -p /tmp/flatbuffers-1.11.0/build && cd /tmp/flatbuffers-1.11.0/build && cmake .. && make && make install

# Setup matching container user to allow for UI
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    mkdir -p /etc/sudoers.d && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

# Install android studio
RUN echo "#!/bin/bash\ncd /volume\ntmux new-session -s dev '/bin/bash'" > /run.sh && chmod a+x /run.sh
ENTRYPOINT /run.sh

ENV HOME /home/developer
USER developer 

