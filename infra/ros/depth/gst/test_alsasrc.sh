
# See https://thiblahute.github.io/GStreamer-doc/alsa-1.0/alsasrc.html?gi-language=c
gst-launch-1.0 alsasrc device=hw:2 ! audioconvert ! vorbisenc ! oggmux ! filesink location=alsasrc.ogg
