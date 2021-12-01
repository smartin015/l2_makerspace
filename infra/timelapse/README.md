# C++ example application working!

```

# Install gstreamer
# see also https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/accelerated_gstreamer.html#wwpID0E0R40HA
sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
sudo apt-get install libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-good1.0-dev \
  libgstreamer-plugins-bad1.0-dev

# Python 3.8 required for http.server specific serving directory in run.sh
sudo apt -y install python3.8

# Make a ramdisk to reduce USB traffic to the SSD
mkdir -p /tmp/ramdisk && chmod 777 /tmp/ramdisk
sudo mount -t tmpfs -o size=128m myramdisk /tmp/ramdisk

# Install paho-mqtt C++ libraries
sudo apt-get install build-essential gcc make cmake cmake-gui cmake-curses-gui git doxygen graphviz libssl-dev
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_WITH_SSL=ON
sudo cmake --build build/ --target install
sudo ldconfig
cd ..
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
sudo cmake --build build/ --target install
sudo ldconfig

# Build the C++ file (can also run build.sh, but including initial command here for posterity)
cd .../l2_makerspace/infra/timelapse
gcc network_av.cc -o network_av -lstdc++ -lpaho-mqtt3c -lpaho-mqtt3cs -lpaho-mqtt3a -lpaho-mqtt3as `pkg-config --cflags --libs gstreamer-1.0`

# Source the aivero environment - note it's modified per .../infra/ros/depth/gst/README.md
source /opt/aivero/rgbd_toolkit_armv8/aivero_environment.h

# Run the binary from the ramdisk directory so the files are output there
cd /tmp/ramdisk && $PATH_TO_REPO/infra/timelapse/network_av

# In a separate tab, run simple webserver
cd /tmp/ramdisk && python3 -m http.server 8080

# Browse to <IP>:8080 for video!
```


Can also use VLC to receive the stream. To record headless:
```
cvlc -vvv http://192.168.1.8:8080/playlist.m3u8 --sout file/ts:stream.mpg

# Playback
vlc stream.mpg
```
