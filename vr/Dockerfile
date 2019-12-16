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
RUN apt-get update && apt-get install -y --fix-missing libxcursor-dev libxinerama-dev libxrandr-dev libxi-dev libasound-dev libpulse-dev libgl1-mesa-dev && rm -rf /var/lib/apt/lists/*

COPY README.md /
CMD ["./Godot_v3.2-beta3_x11.64"]
