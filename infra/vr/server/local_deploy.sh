#!/bin/bash
docker run -it --rm --name=vr -p 44444:44444/udp -p 4243:4243/tcp l2vr-server:latest
