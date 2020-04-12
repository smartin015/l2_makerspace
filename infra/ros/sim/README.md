# L2 Simulation

## Webots

We prefer Webots simulator to Gazebo as it's more compatible with ROS2 and works decently out-of-the-box (even when running on a Chromebook).

## Synchronization with VR

The goal is to have a hosted mixed-reality VR environment where multiple physical and simulated spaces can be overlayed during the same VR session. This allows for a persistent virtual makerspace environment where you can have multiple simulations running and observable (e.g. for parallel reinforcement learning training on a robot in simulation) and also physical sensors to add realism when not in the physical space.

In order to do this, we must synchronize between the running Webots simulation and Godot. For a first attempt, we'll ensure that the [world file](https://cyberbotics.com/doc/reference/webots-world-files) for each running simulation is also loaded into VR, and that all joint states and movable entity positions are published to a ROS topic that the VR server can read.

These topics will be namespaced to the world to prevent collisions with other simulations.

### Dynamic environments

Future dynamic environments (where Webots nodes are created and removed) are possible with a Supervisor node and [reflection calls](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_get_mf_node), but this is significant added complexity and will be followup work. 
