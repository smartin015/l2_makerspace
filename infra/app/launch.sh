#!/bin/bash
docker run -it --rm -v $(pwd)/src:/usr/src/app/src l2app:latest -v $(pwd)/node_modules:/usr/src/app/node_modules /bin/bash
