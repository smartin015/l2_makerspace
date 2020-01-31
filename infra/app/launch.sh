#!/bin/bash
docker run -it --rm -v $(pwd)/src:/usr/src/app/src -v $(pwd)/node_modules:/usr/src/app/node_modules l2app:latest /bin/bash
