#!/bin/bash
docker run -it --rm --name=vr_server --net l2 -p 44444:44444/udp -p 4243:4243/tcp -p 4242:4242/udp l2vr-server:latest
