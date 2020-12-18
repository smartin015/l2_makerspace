ARG L2PKG=l2_ar3
ARG L2NODES="node.py moveit.py"
FROM l2moveit
FROM l2base-pre
FROM l2base-post-sim
COPY --from=0 /moveit/install /moveit/install
COPY --from=1 /node/install /node/install
RUN echo 'source "/moveit/install/setup.bash"' >> /root/.bashrc
