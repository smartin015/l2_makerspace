ARG L2PKG=l2_storage
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post

RUN apt-get update && apt-get install --no-install-recommends -y python3-pip && pip3 install psycopg2-binary watchdog && apt-get clean
COPY --from=0 /node/install /node/install
