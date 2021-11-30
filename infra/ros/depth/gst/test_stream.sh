# v4l2src device="/dev/video0"
gst-launch-1.0 -v videotestsrc ! videoconvert ! clockoverlay ! \
	videoscale ! video/x-raw,width=640, height=360 !  x264enc bitrate=256 ! video/x-h264,profile=\"high\" ! \
	mpegtsmux ! hlssink playlist-root=http://jetson1:8080 location=segment_%05d.ts target-duration=5 max-files=5
