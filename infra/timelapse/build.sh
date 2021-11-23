#!/bin/bash
gcc network_av.cc -o network_av -lstdc++ -lpaho-mqttpp3 -lpaho-mqtt3as `pkg-config --cflags --libs gstreamer-1.0`
