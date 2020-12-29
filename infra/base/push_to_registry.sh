#!/bin/bash

if [ "$#" -eq 0 ]; then
    echo "Usage ./push_to_registry <hostname>"
    exit 1
fi

docker tag l2base $1:5000/l2base
docker image push $1:5000/l2base
docker tag l2base-pre $1:5000/l2base-pre
docker image push $1:5000/l2base-pre
docker tag l2base-post $1:5000/l2base-post
docker image push $1:5000/l2base-post
