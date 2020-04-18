ARG L2PKG=l2_example
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post

RUN pip3 install docker pyyaml

COPY --from=0 /node/install /node/install
