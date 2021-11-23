# C++ example application working!

```
// Make a ramdisk to reduce USB traffic to the SSD
mkdir -p /tmp/ramdisk && chmod 777 /tmp/ramdisk
sudo mount -t tmpfs -o size=64m myramdisk /tmp/ramdisk

// Install paho-mqtt C++ libraries
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

// Build the C++ file (can also run build.sh, but including initial command here for posterity)
cd .../l2_makerspace/infra/timelapse
gcc network_av.cc -o network_av -lstdc++ -lpaho-mqtt3c -lpaho-mqtt3cs -lpaho-mqtt3a -lpaho-mqtt3as `pkg-config --cflags --libs gstreamer-1.0`

// Source the aivero environment - note it's modified per .../infra/ros/depth/gst/README.md
source /opt/aivero/rgbd_toolkit_armv8/aivero_environment.h

// Run the binary from the ramdisk directory so the files are output there
cd /tmp/ramdisk && $PATH_TO_REPO/infra/timelapse/network_av

// In a separate tab, run simple webserver
cd /tmp/ramdisk && python3 -m http.server 8080

// Browse to <IP>:8080 for video!

