sudo docker run --rm -e DISPLAY --device /dev/snd -v /tmp/.X11-unix:/tmp/.X11-unix -v ${pwd}/projects:/projects -it l2vr:latest
