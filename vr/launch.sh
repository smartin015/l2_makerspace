#!/bin/bash
docker run -e DISPLAY --device /dev/snd -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd)/projects:/projects -v $(pwd)/home:/home/developer --device /dev/bus/usb -it l2vr:latest
