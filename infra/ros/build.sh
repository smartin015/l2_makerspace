#!/bin/bash
ROOT=$(pwd)
DIRS=$(find . -mindepth 1 -maxdepth 1 -type d -printf '%f\n')
for d in $DIRS; do
  echo "Building $d as l2$d"
  cd $ROOT/$d && docker build --tag "l2$d:latest" .
done
cd $ROOT
