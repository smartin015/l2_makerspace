#!/bin/bash

docker run -it -p 2221:2222  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --privileged testrpnb:latest
