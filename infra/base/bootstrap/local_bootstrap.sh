#!/bin/bash
echo "Make sure you're running this script from the root of l2_makerspace"

echo "TODO Install the latest version of Ubuntu on your local server"

echo "Starting docker container ensemble (handing over to docker-compose)"
docker-compose \
  -f infra/ros/bridge/docker-compose.yml \
  -f infra/app/docker-compose.yml \
  -f infra/ros/storage/docker-compose.yml \
  -f infra/ros/sim/docker-compose.yml \
  -f infra/ros/tf_fwd/docker-compose.yml \
  -f infra/vr/server/docker-compose.yml \
  up \
  gz tf_fwd bridge_local
# client app storage db vr_server sdf2
echo "TODO Starting the local DB container"
echo "TODO vdi/ to start up the virtual desktop server container"
echo "TODO rpi/ to start up the PXE / netboot container"
echo "TODO to start up the app handler container"
echo "TODO to start up a local DNS server to resolve the above containers"
