```
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


