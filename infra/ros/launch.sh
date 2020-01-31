#!/bin/bash
docker-compose -f ../base/base-compose.yml -f $1/docker-compose.yml up
