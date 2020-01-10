#!/bin/bash
sudo x11docker --gpu -- "-v $(pwd)/projects:/projects -v $(pwd)/home:/home/developer --device /dev/bus/usb" l2vr:latest
