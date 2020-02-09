# Level 2 Makerspace Infra

Code and configuration for infrastructure deployable into an L2 makerspace.

## Capabilities

### vr

In the `vr/` folder is a number of software tools and programs for running a VR makerspace. Details at [./vr/README.md]

### virtual\_desktop

The `virtual_desktop/` folder is for virtual desktop infra - virtual desktops are created for individual projects, and 
future work will allow these virtual desktops to migrate around the physical (or virtual) makerspace as their
user moves from place to place. Details at [./vdi/README.md]

### ros

The `ros` folder contains all the ROS2 nodes which control aspects of the makerspace. For instance, `ros/storage/` contains a docker-compose environment for running a postgres DB hooked up to a ROS2 node, which can
store data at rest for access over the ROS2 network.

### embedded

`embedded/raspi/` simulates a raspberry pi in a Docker container - useful for testing software to be deployed on multiple raspberry pi's
in a physical makerspace.

`embedded/esp8266/` contains source code for the esp8266 microcontroller.

### app

The `app/` folder hosts a native and web compatible app for managing makerspace projects that plugs into the other capabilities
in the space.


### browser

`browser/` is a chrome browser extension that facilitates L2 making while e.g. browsing or purchasing new parts for a project.
