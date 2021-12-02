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
  PIDLINE=$(ps axf | grep "http.server" | grep -v grep)
  echo $PIDLINE
  read -p "Kill this process and start a new one? " -n 1 -r
  echo    # (optional) move to a new line
  if [[ $REPLY =~ ^[Yy]$ ]]
  then
    KILLCMD=$(echo "$PIDLINE" | awk '{print "kill -9 " $1}')
    echo $KILLCMD | sh
    echo "Kill't"
  else
    echo "Goodbye"
    exit 1
  fi
}

echo "Starting HTTP server in background"
$HTTP_CMD &
HTTP_PID=$!
echo "PID $HTTP_PID"


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

echo "Shutting down HTTP daemon (pid $HTTP_PID)"
kill $HTTP_PID

echo "Unmounting ramfs"
sudo umount $RAMDISK

echo "Done"
