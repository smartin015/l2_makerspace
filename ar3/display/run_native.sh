#!/bin/bash
flatc --cpp --gen-object-api -o ./lib/proto ./lib/proto/*.fbs
flatc -o ./data -b lib/proto/meta.fbs ./assets/metadata.json
pio run -e native
.pio/build/native/program
