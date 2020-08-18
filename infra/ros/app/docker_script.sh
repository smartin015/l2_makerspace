#!/bin/bash
docker run -it --rm -v $(pwd)/scripts:/scripts l2base:latest $@
