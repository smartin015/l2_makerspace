ARG L2PKG=l2_app
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post
COPY --from=0 /node/install /node/install
COPY ./src /src

FROM l2base:latest
