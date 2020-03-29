# Level 2 Makerspace

http://l2making.com/

## Overview

This repository contains design docs, software tools, work tracking, and other digital assets to support a Level 2 Makerspace.

See [docs/design.md](https://github.com/smartin015/l2_makerspace/blob/master/docs/design.md) for more detail on the overall mission and planning.

## Docker

This repo relies heavily on Docker containers to keep everything modular and self-contained.

See ./base for base image (available publicly for download at gcr.io/l2-making/base)

### Tricky Details

* ROS2 uses DDS as a backing networking protocol - and it [has issues](https://answers.ros.org/question/296828/ros2-connectivity-across-docker-containers-via-host-driver/) when your run docker containers with the "host" network mode. For simplicity, always use `network_mode: bridge` when setting up docker nodes via docker-compose.
* Docker support for GPU acceleration is device-specific: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#Intel
* A ROS2 subscriber can receive a topic message only if its QOS profile is compatible with the QOS profile of the publisher - [src](https://answers.ros.org/question/304946/ros2-retrieving-qos-settings-for-a-topic/)
* The default docker "bridge" network doesn't allow lookups by container name, only by IP address - must use a user-defined container for name resolution
