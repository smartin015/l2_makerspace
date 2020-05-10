# L2 Docker/ROS setup

L2 infrastructure relies very heavily on Docker to keep things portable and to allow for 
consistency when running many different software stacks (e.g. ROS and FreeCAD can run
using the same launch framework of docker-compose).

## ROS2 configuration

`rclpy` is the name of the framework for ROS2 python nodes - the [docs for rclpy](http://docs.ros2.org/eloquent/api/rclpy/) are handy, but perhaps more handy are the test files in the rclpy github repo (e.g. [test_time.py](https://github.com/ros2/rclpy/blob/master/rclpy/test/test_time.py) I refer to regularly to understand how to interact with ROS2 time data).

The python framework for ROS2 requires two specific files to configure a python node: setup.py and package.xml. Interestingly, these two files mostly encode the same data; I consider setup.py as largely redundant to the ros package config, so it's templated and not actually configured for L2 nodes. This is done via docker ONBUILD invocations and scripts in the .../infra/base/templates/ folder - more on that later.

The package XML file is included though, in case dependencies of the ROS2 package are different from module to module. See .../infra/ros/example/package.xml for an example package file.

## ONBUILD and base containers

The very base container used for L2 is at .../infra/base/Dockerfile - it contains the bare minimum to make a ROS node, plus custom L2 message types defined in .../infra/base/l2_msg/.

However, typically this container isn't actually used directly. There are `./pre` and `./post` folders; the `./pre` folder dockerfile builds the ROS module (including seting up templated files like `setup.py`) and the `./post` folder dockerfile is the slimmer environment where the node actually runs. 

Check out `.../infra/ros/example/Dockerfile for the invocation order of these containers. As of May 2020, the file looks like:

```
ARG L2PKG=l2_example
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post
COPY --from=0 /node/install /node/install
```

The two args at the start are used in `l2base-pre` to construct the approprate templates and build the right nodes. Then `l2base-post` is used as the final image base and the `/node/install` directory is copied to it (which contains the built ROS2 package).
