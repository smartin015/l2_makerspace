# Makerspace Docker VDI

## Purpose

This directory contains example code for launching virtual desktop infrastructure (VDI) using docker containers (original source: https://github.com/dtpnk/docker-vdi). 

The intent is to automatically start up one virtual desktop per makerspace project, which minimizes cross-project clutter and allows for other future features, such as:

* Automatic desktop switching when switching projects
* "follower" desktop experience that moves with the user through the makerspace, across different displays
* Automatic recording buffer of desktop work

## Instructions

* Make sure nxserver is not running on the host (`sudo /usr/NX/bin/nxserver --shutdown`)
* Run `docker-compose up` to start a single VDI server
* Username and password are determined inside the yml file.
* Run `/usr/NX/bin/nxplayer`. It should detect the new host and allow you to connect. `--config` may prove handy as a way to automatically run the player.
