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
* A ROS2 subscriber can receive a topic message only if its QOS profile is compatible with the QOS profile of the publisher - [src](https://answers.ros.org/question/304946/ros2-retrieving-qos-settings-for-a-topic/). Nothing tells you this; the messages just aren't received.
* The default docker "bridge" network doesn't allow lookups by container name, only by IP address - must use a user-defined container for name resolution
* ROS message transport (Fast RTPS) breaks if the same PID and IP address are used more than once (https://github.com/ros2/rmw_fastrtps/issues/349)
* If long-running nodes exit with error (137), it's an OOM. `docker ps -a` to find termination time, then `less /var/log/kern.log` and go to the time of death to correlate. 
* ROS nodes don't by default allow you to make nested async calls (e.g. publish from a subscriber callback). A `ReentrantCallbackGroup` must be used (see [vr node](https://github.com/smartin015/l2_makerspace/blob/master/infra/ros/vr/node.py) for an example that also supports service call deadlines).
* Webots ros integration requires the following environment variables to run (as well as sourcing the ROS setup.bash file):
  *  `export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller"`
  *  `export PYTHONPATH="$PYTHONPATH:$WEBOTS_HOME/lib/controller/python36"`
  *  `export WEBOTS_HOME=/usr/local/webots`
* As of 2020Q2, Webots doesn't export debian packages for ROS2 Eloquent. If using Crystal, the examples need to be modified to remove QoS settings on publisher definitions:
  *  `/opt/ros/crystal/lib/python3.6/site-packages/webots_ros2_examples/example_controller.py`
  *  `vim /opt/ros/crystal/lib/python3.6/site-packages/webots_ros2_core/webots_node.py`
* You can build webots from source for eloquent; this is probably easier.
* Webots ROS2 Node class has a timer that calls a `timer_callback` callback - accidentally overriding this causes the controller to halt. There's also naming collisions in rclp if you name something "Sequence"
* Sometimes docker-compose takes a looong time to start, with no console output. This was because there was low entropy on the server in order to open a secure connection (see [here](https://github.com/docker/compose/issues/6552))

