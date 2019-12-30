# Raspberry pi image emulation

## Purpose

Simulate a raspberry pi for the purpose of testing e.g:

* PXE booting to a server hosted image
* Running X display code to e.g. connect to a virtual desktop instance

## Instructions

Testing example: 

`docker run -it -p 2222:2222 --privileged hannseman/raspbian`

## References

https://github.com/ferrarimarco/docker-pxe

https://github.com/hannseman/docker-raspbian
