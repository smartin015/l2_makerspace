FROM bitnami/minideb:jessie 
#gcr.io/l2-making/base

ENV GODOT_VERSION "3.2-beta3"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    wget \
    unzip \
    python \
    python-openssl \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://downloads.tuxfamily.org/godotengine/3.2/beta3/Godot_v3.2-beta3_x11.64.zip \
    && wget https://downloads.tuxfamily.org/godotengine/3.2/beta3/Godot_v3.2-beta3_export_templates.tpz \
    && mkdir ~/.cache \
    && mkdir -p ~/.config/godot \
    && mkdir -p ~/.local/share/godot/templates/${GODOT_VERSION}.stable \
    && unzip Godot_v3.2-beta3_x11.64.zip \
    && mv Godot_v3.2-beta3_x11.64.zip /usr/local/bin/godot \
    && unzip Godot_v3.2-beta3_export_templates.tpz \
    && mv templates/* ~/.local/share/godot/templates/${GODOT_VERSION}.stable \
    && rm -f Godot_v3.2-beta3_x11.64.zip Godot_v3.2-beta3_export_templates.tpz

# TODO merge with earlier
RUN apt-get update && apt-get install -y --fix-missing sudo libxcursor-dev libxinerama-dev libxrandr-dev libxi-dev libasound-dev libpulse-dev libgl1-mesa-dev default-jdk libz1 libncurses5 libstdc++6 lib32stdc++6 lib32z1 && rm -rf /var/lib/apt/lists/*

RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    mkdir -p /etc/sudoers.d && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

USER developer
ENV HOME /home/developer		

RUN wget https://dl.google.com/dl/android/studio/ide-zips/3.5.3.0/android-studio-ide-191.6010548-linux.tar.gz && \
     tar -xvf android-studio-ide-191.6010548-linux.tar.gz && rm android-studio-ide-191.6010548-linux.tar.gz

COPY README.md /
CMD ["./Godot_v3.2-beta3_x11.64"]
