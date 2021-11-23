#!/bin/bash
gcc network_av.cc -o network_av -lstdc++ `pkg-config --cflags --libs gstreamer-1.0`
