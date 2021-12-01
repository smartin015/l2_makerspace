#!/bin/bash

PORT=8080
RAMDISK=/tmp/ramdisk
RGBD_TOOLKIT_ENV=/opt/aivero/rgbd_toolkit_armv8/aivero_environment.sh

mountpoint --quiet $RAMDISK 2> /dev/null && {
  echo "Ramdisk already mounted; clearing old *.ts and *.m3u8 files"
  rm -rf $RAMDISK/*.m3u8 $RAMDISK/*.ts
} || {
  echo "Ramdisk not present at $RAMDISK; creating one"
  mkdir -p $RAMDISK && chmod 777 $RAMDISK
  sudo mount -t tmpfs -o size=128m network_av $RAMDISK
}

HTTP_CMD="python3.8 -m http.server -d $RAMDISK $PORT"
ps -alfe | grep "$HTTP_CMD" | grep -qv grep && {
  echo "HTTP server already started"	
} || {
  echo "HTTP server not started; starting daemon"
  $HTTP_CMD &
}

if [ -f "$RGBD_TOOLKIT_ENV" ]; then
  echo "Sourcing gstreamer RGBD toolkit for realsense cameras"
  . $RGBD_TOOLKIT_ENV

  echo "Fetching realsense device ID..."
  DEVICE_ID=$(rs-enumerate-devices -s | awk -F '  +' '/D435/ {print $2}')

  echo "Device ID is $DEVICE_ID. Starting RGB network_av..."
  ./network_av -p rgb -s $DEVICE_ID -o $RAMDISK/
elif lsusb | grep "Yeti Stereo Microphone"; then
  echo "Starting network_av in microphone mode..."
  # TODO detect proper HW number and use it here
  ./network_av -s mic -d $DEVICE_ID -o $RAMDISK/
else 
  echo "ERR: Could not detect any supported devices for streaming!"
fi

