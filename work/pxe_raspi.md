# PXE Rasberry PI simulation environment

## Milestone 1

Create two separate docker containers:

1. A PXE server container which hosts a Raspbian image
2. A QEMU emulator which runs a raspi-like environment with network boot enabled

With the following behaviors:

1. Start PXE server container
2. Start virtual raspi container
3. raspi container connects to PXE server container and boots from the image

## Resources

https://github.com/smartin015/l2_makerspace/tree/master/rpi

