#!/bin/bash
export SERIAL=819112070701
# demux.src_depth ! queue ! colorizer near-cut=300 far-cut=700 ! videoconvert ! glimagesink \
gst-launch-1.0 realsensesrc serial=$SERIAL timestamp-mode=clock_all enable-color=true  ! \
	  rgbddemux name=demux demux.src_color ! queue ! videoconvert ! omxh264enc ! \
  'video/x-h264, stream-format=(string)byte-stream' ! h264parse ! \
  mpegtsmux ! hlssink playlist-root=http://192.168.1.8:8080 location=/tmp/ramdisk/segment_%05d.ts target-duration=5 max-files=5



