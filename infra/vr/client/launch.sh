#!/bin/bash
sudo x11docker --gpu -- "--rm -v $(pwd):/client -v $(pwd)/../home:/fakehome/$(whoami) --device /dev/bus/usb" l2vr:latest /Godot_v3.2-stable_x11.64 --editor /client/project.godot
