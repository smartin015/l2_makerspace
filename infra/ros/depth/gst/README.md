<<<<<<< HEAD
```
=======
# Requirements & Setup

These scripts **MUST** be set up within an installation of the NVIDIA jetson nano image - jetson nanos running Ubuntu
or other distro fail to install the proper gstreamer plugins to allow for hardware-based video encoding.

### NVIDIA Jetson gstreamer plugins

Follow procedure at https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/accelerated_gstreamer.html#wwpID0E0R40HA

TL;DR:

```
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt-get update
sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
sudo apt-get install libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-good1.0-dev \
  libgstreamer-plugins-bad1.0-dev
```

### Aivero realsense toolkit 
Original installation instructions at https://gitlab.com/aivero/legacy/public/aivero-rgbd-toolkit.

Download the latest armv8 release - currently [this one](https://drive.google.com/u/0/uc?id=1VoBx2SES10AWMiHBqR-gwFW8vX-beZvc&export=download) as of 2021Q4.

Transfer it to the jetson nano (e.g. via SCP) and then extract it - the archive must be extracted relative to root for the right folder locations:

```
sudo mkdir -p  /opt/aivero && sudo chown $USER:USER /opt/aivero
tar -C / -xvf aivero_rgbd_toolkit_master.tar.gz
```

To enable use of both the NVIDIA and the aivero GST plugins, we need to edit the environment so both paths are included.
Open /opt/aivero/rgbd_toolkit_armv8/aivero_environment.sh and replace the GST_PLUGIN_PATH line with:

```
export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0:$PREFIX/lib/gstreamer-1.0
```

# Manual testing

Note: fetch the serial number via `rs-enumerate-devices | less`

```
source /opt/aivero/rgbd_toolkit_armv8/aivero_environment.sh 

export SERIAL=819612071634
gst-launch-1.0 realsensesrc serial=$SERIAL timestamp-mode=clock_all enable-color=true  ! \
  rgbddemux name=demux \
  demux.src_depth ! queue ! colorizer near-cut=300 far-cut=700 ! videoconvert ! glimagesink \
  demux.src_color ! queue ! videoconvert ! glimagesink

```

# Basic streaming

```
# Create a ramdisk to reduce chatter on the bus (assuming we're using a USB SSD)
# Following tutorial at https://www.linuxbabe.com/command-line/create-ramdisk-linux
# Experimentally, video streaming seems to use about 15M of space.
sudo mkdir /tmp/ramdisk && chmod 777 /tmp/ramdisk
sudo mount -t tmpfs -o size=50m myramdisk /tmp/ramdisk

# Test basic compressed web streaming
./test_stream.sh

# in a different pane
python3 -m http.server 8080

# in a browser, go to http://jetson1:8080 and play the video
```


# Custom gst plugin to convert depth camera image to RVL

Pulling from https://github.com/jackersson/gst-python-plugins

`gstplugin_py` is the default example plugin

```
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD/venv/lib/gstreamer-1.0/:$PWD/gst/
gst-inspect-1.0 gstplugin_py
gst-launch-1.0 videotestsrc ! gstplugin_py int-prop=100 float-prop=0.2 bool-prop=True str-prop="set" ! fakesink

```

`rvlencode` converts to run-variable-length encoding


# Realsense data access via Aivero RGBD toolkit
Installation instructions at https://gitlab.com/aivero/legacy/public/aivero-rgbd-toolkit

Note: serial number can be fetched via `rs-enumerate-devices | less`

```
source /opt/aivero/rgbd_toolkit_armv8/aivero_environment.sh 

export SERIAL=819612071634
gst-launch-1.0 realsensesrc serial=$SERIAL timestamp-mode=clock_all enable-color=true  ! \
  rgbddemux name=demux \
  demux.src_depth ! queue ! colorizer near-cut=300 far-cut=700 ! videoconvert ! glimagesink \
  demux.src_color ! queue ! videoconvert ! glimagesink

```


