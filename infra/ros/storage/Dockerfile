ARG L2PKG=l2_storage
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post

RUN pip3 install psycopg2-binary
COPY --from=0 /node/install /node/install
