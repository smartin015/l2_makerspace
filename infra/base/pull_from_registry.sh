#!/bin/bash

if [ "$#" -eq 0 ]; then
    echo "Usage ./pull_to_registry <hostname>"
    exit 1
fi

docker image pull $1:5000/l2base
docker image pull $1:5000/l2base-pre
docker image pull $1:5000/l2base-post
