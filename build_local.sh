#!/bin/bash
echo "Building all containers for local development..."

# Build base images
INFRADIR=$(dirname "$0")/infra
set -x
$INFRADIR/base/build.sh

# Build ros nodes
for tag in storage sim bridge vr
do
  docker build --tag "l2$tag" -f $INFRADIR/ros/$tag/Dockerfile $INFRADIR/ros/$tag/
done
