# Bootstrapping a new L2 makerspace

This doc outlines how to spin up a new physical (and virtual) Level 2 space for people to use.

Note: This hasn't really been attempted before, so consider these steps mostly untested until told otherwise.

## Requirements/Materials

### One-offs

Several odds and ends are needed which don't scale with the complexity of your space:

- A Google cloud developer account for starting up containers via GKE that need to be externally visible.
  Some of this may be hosted by the original L2 makerspace, so reach out to contact@fabricate.io before going much further.
- A github user used or forking this repository to modify it to suit your space (and to issue pull requests, *wink wink*)
- A local server handle large, ephemeral data storage, data at rest, and high-bandwidth communication. 
  - Recommend something like >1T of storage, >8G ram, a decent (i7?) CPU and a recent (<2yr old) GPU for simulation and machine learning. 
  - You can skimp on the GPU or even the storage and instead rely on GCE storage and GPU/TPU, but that can 
    get pricy in the long run. It's handy to have a local source.
- A wired LAN network with >1Gbps network switch with as many ports as you have displays in the makerspace.

### Packages 

Depending on the size of your space and your ambition, you can set up a number of displays and other technologies in
the space. Here are a few "package" options:

Smart workspace monitor:

- 1x raspberry pi
- 1x HDMI (or other compatible video cable)
- 1x monitor
- 1x LAN cable (to run to ethernet switch)
- 1x power supply (5V wall to usb micro supply, >2A supply)

Robotic / teleoperated workstation

- 1x robotic arm (TODO spec this out)
- 1x Intel Realsense depth camera
- 1x USB C to A cable (from Realsense to raspi)
- 1x raspberry pi (or preferably Odroid XU4 or stronger)
- 1x LAN cable (to run to ethernet switch)

### Bootstrapping

#### Server

The server hosts several containers which may be tricky to set up right. 
Use `local_bootstrap.sh` in the base directory of this repo for this to be set up automatically.

#### Google cloud

The cloud hosts several containers which may be tricky to set up right. 
Use `cloud_bootstrap.sh` in the base directory of this repo for this to be set up automatically.

#### Embedded displays

Note: the server should be set up with VD

1. Configure each raspi for netboot
1. Plug them into the network via wired LAN and their 
   companion displays. They should automatically boot, install the display image, and connect to
   the local server.
