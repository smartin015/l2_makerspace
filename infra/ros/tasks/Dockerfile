ARG L2PKG=l2_tasks
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post

RUN apt update && apt install --no-install-recommends -y python3-pip && pip3 install todoist-python && apt clean
COPY --from=0 /node/install /node/install
