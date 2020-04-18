# L2 Simulation

## Webots

We prefer Webots simulator to Gazebo as it's more compatible with ROS2 and works decently out-of-the-box (even when running on a Chromebook).

## Synchronization with VR

The goal is to have a hosted mixed-reality VR environment where multiple physical and simulated spaces can be overlayed during the same VR session. This allows for a persistent virtual makerspace environment where you can have multiple simulations running and observable (e.g. for parallel reinforcement learning training on a robot in simulation) and also physical sensors to add realism when not in the physical space.

In order to do this, we must synchronize between the running Webots simulation and Godot. For a first attempt, we'll ensure that the [world file](https://cyberbotics.com/doc/reference/webots-world-files) for each running simulation is also loaded into VR, and that all joint states and movable entity positions are published to a ROS topic that the VR server can read.

These topics will be namespaced to the world to prevent collisions with other simulations.

### Dynamic environments

Future dynamic environments (where Webots nodes are created and removed) are possible with a Supervisor node and [reflection calls](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_get_mf_node), but this is significant added complexity and will be followup work. 

### Multiple environments

It may be desirable in the future to run multiple independent environments simultaneously (e.g. for parallel reinforcement learning). This can be done with docker's swarm mode; the swarm master launches `sim_worker` nodes on a number of connected worker hosts 

### Startup flow

1. User (or automation) launches `sim_worker` container (a ros2 node), with a configurable `l2_msgs/Simulation` message:
   * ROS namespace
   * World config file (or reference name to be looked up via ros2 storage node)
   * Status topic path
1. `sim_worker` launches Webots and periodically reports its `Simulation` message on the status topic path
1. User launches one or more `controller` ROS nodes (Webots `<extern>` style)
   * In multi-robot simulation environments, can specify `--webots-robot-name` to assign a controller to a specific robot
1. `controller` nodes interact with the Webots instance and publish e.g. `JointState` and `TF` messages
1. `vr` ROS node reads sim_worker's published `Simulation` message, fetches the same config from storage node and pushes it to the VR server
1. VR server caches the simulation state and forwards to all VR clients (including to any clients connecting after)
1. VR clients load the config, subscribe to all relevant topics, and update their state based on the published topics from the `controller` nodes
1. When the simulation is terminated (`sim_worker` and all `controller`s brought down), the `vr` ROS node detects the absence of [topic publishers](http://docs.ros2.org/crystal/api/rclpy/api/node.html#rclpy.node.Node.count_publishers) and tells the VR server to end the simulation
1. The VR server forwards the end of simulation to all VR clients, who clean up their state.

### Manual test with example controller

Create the volume for unix socket comms (and the l2 network if not already exists):

```
docker volume create webots
docker network create l2
```

Start the webots server container 

```
docker run -v webots:/tmp -v $(pwd)/single_motor.wbt:/single_motor.wbt --volume=/tmp/.X11-unix:/tmp/.X11-unix   
--device=/dev/dri:/dev/dri   --env="DISPLAY=$DISPLAY"   -it --rm l2sim /webots_ros2/install/webots_ros2_desktop/share/webots_ros2_desktop/webots/web
ots --no-sandbox --stdout --stderr /single_motor.wbt
```

Start the controller container:

```
docker run -it --rm -v webots:/tmp --net=l2 l2sim /bin/bash
export WEBOTS_HOME=/webots_ros2/install/webots_ros2_desktop/share/webots_ros2_desktop/webots/
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller
source /ros_entrypoint.sh
ros2 run l2_sim example_controller --ros-args -p synchronization:=False
```

Exec another ros node and listen to the topic

```
docker run -it --rm --net=l2 l2base /bin/bash
source /ros_entrypoint.sh
ros2 topic echo /joint_state
```

### Client simulation to server

Can [install a VPN](https://www.cyberciti.biz/faq/howto-setup-openvpn-server-on-ubuntu-linux-14-04-or-16-04-lts/) to allow for ROS2 comms between server and client.


