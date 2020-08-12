ARG L2PKG=l2_example
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post

RUN apt update && apt install --no-install-recommends -y python3-pip \
 && apt clean && pip3 install docker pyyaml watchdog

COPY --from=0 /node/install /node/install
