#!/bin/bash
ROOT=$(pwd)
cd $ROOT/example && docker build --tag l2example:latest .
cd $ROOT/depth_camera && docker build --tag l2depth:latest .
cd $ROOT/gazebo && docker build --tag l2sim:latest .
cd $ROOT/tasks && docker build --tag l2tasks:latest .
cd $ROOT/storage && docker build --tag l2storage:latest .
cd $ROOT/bridge && docker build --tag l2bridge:latest .
cd $ROOT/tensorflow && docker build --tag l2tensorflow:latest .
cd $ROOT
