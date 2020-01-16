# PXE Rasberry PI simulation environment

## Objective

Create two separate docker containers:

1. A PXE server container which hosts a Raspbian image
2. A QEMU emulator which runs a raspi-like environment with network boot enabled

With the following behaviors:

1. Start PXE server container
2. Start virtual raspi container
3. raspi container connects to PXE server container and boots from the image

## Details

Testing example for running a raspbian emulator in a docker container: 

`docker run -it -p 2222:2222 --privileged hannseman/raspbian`

Testing example of running a PXE server in a docker container: 

`docker run -it --rm --net=host ferrarimarco/pxe`

The difficult part so far has been tweaking the raspbian container to not require the raspbian image, while still having viable networking and starting with network boot.

## References

https://github.com/ferrarimarco/docker-pxe

https://github.com/hannseman/docker-raspbian
