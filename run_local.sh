#!/bin/bash
echo "TODO ensure all images are built"
echo "TODO if DISPLAY not set and no X servers running, start a virtual X display and set it"
echo "Make sure you're running this script from the root of l2_makerspace"
echo "Starting docker container ensemble (handing over to docker-compose)"
INFRADIR=$(dirname "$0")/infra
export COMPOSE_PROJECT_NAME=l2

APP="app"
VR="vr_server"
if [ $1 = "local" ]; then
  APP="app_local"
  VR=""
  echo "Starting in 'local' mode; App runs from src/ directory and VR server must be started manually"
fi

set -x

docker-compose \
  --project-directory $(pwd) \
  -f $INFRADIR/app/docker-compose.yml \
  -f $INFRADIR/ros/bridge/docker-compose.yml \
  -f $INFRADIR/ros/tasks/docker-compose.yml \
  -f $INFRADIR/ros/storage/docker-compose.yml \
  -f $INFRADIR/ros/supervisor/docker-compose.yml \
  -f $INFRADIR/ros/vr/docker-compose.yml \
  -f $INFRADIR/vr/server/docker-compose.yml \
  up --abort-on-container-exit \
  db storage \
  vr_syncer vr_fwd bridge $VR \
  supervisor \
  $APP bridge_server \

# echo "TODO vdi/ to start up the virtual desktop server container"
# echo "TODO rpi/ to start up the PXE / netboot container"
# echo "TODO to start up a local DNS to resolve the above containers"
