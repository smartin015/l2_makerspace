# Level 2 Makerspace

http://l2making.com/

## Overview

This repository contains design docs, software tools, work tracking, and other digital assets to support a Level 2 Makerspace.

See [docs/design.md](https://github.com/smartin015/l2_makerspace/blob/master/docs/design.md) for more detail on the overall mission and planning.

## History & Current State

This is a passion project which started in late 2019 as a chance to collect several technologies and skills which were fun to learn
and had the potential for really interesting applications in making.

Through 2020, my focus has been on prototyping and proving various concepts, including:

* Templates & basic protocols & frameworks (e.g. ROS2, ./infra/base messages & docker container processes)
* Integrations (e.g. Todoist for creating projects, Docker for launching sequences)
* Early hardware proofs of concept (simulation with Webots and publishing compressed point cloud data from Realsense D435 cameras)
* Visualization (particularly in VR, see ./infra/vr/client)
* Networking (e.g. routing between internal ROS network and external VR/app server)

## Roadmap

* See http://app.l2making.com/#project/TODO

## Support

This software is provided as-is with no guarantee of support. 

However, contributions are welcome - please reach out if you're interested
in helping with development.

### Lessons Learned

The following is a collection of details which were learned over hours of frustration. Please read through these if you're stuck on something, and
add new lessons learned.

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
* Streaming data from Webots requires setting IPC to "host" mode on the docker container; otherwise it exits with a "code 245"
* Ubuntu hides raspi install images for older versions (e.g. 18.04 LTS). You can find them [here](http://cdimage.ubuntu.com/releases/bionic/release/) - but raspbian is the most compatible for running e.g. D435 cameras.
* There's a *ton* of manual effort in setting up a realsense camera on raspi.
  * If you get frame drops, try updating the firmware on the camera. There's some "uvcvideo" kernel module patching stuff that didn't actually work, but could also be helpful. [details](https://eleccelerator.com/wiki/index.php?title=Raspbian_Buster_ROS_RealSense)
* Godot uses `scons` for development. You have to build the binary, then the export templates separately (see [here](https://docs.godotengine.org/en/3.0/development/compiling/compiling_for_x11.html) for linux binary, and [here](https://docs.godotengine.org/en/latest/development/compiling/compiling_for_android.html#building-the-export-templates) for android export templates)
* Super useful librealsense docs around camera intrinsics and projection: [link](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#intrinsic-camera-parameters)
  * Remember to build with android\_arch=armv7 or else the project will fail to export.
* Can generate space skyboxes with [link](http://wwwtyro.github.io/space-3d), but have to convert (see [link](https://www.reddit.com/r/godot/comments/baw27a/how_to_use_spacescape_skybox_in_godot/))
* Using the Godot OVR toolkit, 2D panels (i.e. most tools in L2 VR) are rendered with the viewport size matching the device resolution. This means tools can have different sizes if they're rendered on different devices (or if you resize the window). Setting display resolution to 2880x1600 makes them match.
