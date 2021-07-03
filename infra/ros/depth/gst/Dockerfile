FROM ubuntu:20.04

USER root

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends install \
    build-essential \
    git \
    cmake \
    autoconf \
    automake \
    libtool \
    pkg-config \
    python3.6 \
    python3-pip \
    python3-dev \
    gstreamer-1.0 \
    gstreamer1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-tools \
    libgstreamer1.0-0 \
    libgstreamer1.0-0 \
    libgirepository1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libcairo2-dev \
    gir1.2-gstreamer-1.0 \
    python3-gi \
    gstreamer1.0-python3-plugin-loader \
    && pip3 install -U wheel pip setuptools \
    && pip3 install git+https://github.com/jackersson/gstreamer-python.git#egg=gstreamer-python \
    && rm -rf /var/lib/apt/lists/*


ADD gst gst
ENV GST_PLUGIN_PATH /usr/lib/gstreamer-1.0/:/gst/
CMD gst-inspect-1.0 gstplugin_py
