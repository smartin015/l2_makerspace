#!/bin/bash

# Authorize use of X display
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Optional device path as first argument, to allow for flashing to device from container.
[[ $1 ]] && DEVMAPFLAG="--device=$1:/dev/ttyUSB0" || DEVMAPFLAG=""

# Alternative if the below command doesn't work (uses nvidia-docker)
# docker run --gpus=all --name l2display -it --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw l2display /bin/bash

RESOLVE_PATH=$(pwd)
  # --env="QT_X11_NO_MITSHM=1" \
docker run -v $RESOLVE_PATH:/volume \
  --net host \
  --env="DISPLAY=${DISPLAY}" \
  --env="XAUTHORITY=${XAUTH}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth \
  ${DEVMAPFLAG} \
  -it l2display:latest
