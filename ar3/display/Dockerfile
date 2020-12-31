# For some reason, only l2sim seems to have proper graphis libraries set up right now?
FROM l2sim

MAINTAINER Scott Martin <smartin015@gmail.com>

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y curl git python3 python3-pip bash build-essential cmake gcc-multilib g++-multilib \
		      libsdl2-dev libwebsockets-dev mesa-utils libgl1-mesa-glx && rm -rf /var/lib/apt/lists/*
RUN pip3 install -U platformio 

RUN curl https://codeload.github.com/google/flatbuffers/tar.gz/v1.11.0 -o /tmp/flatbuffers.tar.gz && cd /tmp && tar xvf flatbuffers.tar.gz && mkdir -p /tmp/flatbuffers-1.11.0/build && cd /tmp/flatbuffers-1.11.0/build && cmake .. && make && make install

RUN python3 -m platformio update

ENTRYPOINT /bin/bash
