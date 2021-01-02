#!/bin/bash
flatc --cpp --gen-object-api -o ./lib/proto ./lib/proto/*.fbs && \
flatc -o ./data -b lib/proto/config.fbs ./data/secret.json && \
python3 -m platformio run -e native && \
.pio/build/native/program
