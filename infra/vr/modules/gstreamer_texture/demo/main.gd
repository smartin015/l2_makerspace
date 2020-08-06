extends Node

onready var GStreamer = load("res://bin/gstreamer.gdns")

func _ready():
  var gs1 = Node.new()
  gs1.set_script(GStreamer)
  gs1.image_texture = $HBoxContainer/TextureRect.texture
  # Note: video alignment issues can happen if the final width/height pixel
  # values don't match well with the aspect ratio of the video.
  # If you get werid shifing effects, try 640x480 or 480x480.
  gs1.pipeline = "uridecodebin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm ! videoconvert ! videoscale ! video/x-raw, format=RGB, width=640, height=480 ! appsink name=videosink emit-signals=true sync=true max-buffers=1 drop=false"
  # gs1.asgp = $AudioStreamPlayer.get_stream_playback()
  # gs1.pipeline = "audiotestsrc  wave=0 freq=440 volume=0.2 ! audioconvert ! stereo ! audioconvert ! audio/x-raw,format=F32LE,channels=2,layout=interleaved ! appsink name=audiosink emit-signals=true sync=true max-buffers=3 drop=false throttle-time=100000000"
  gs1.connect("new_caps", self, "_on_GStreamer_new_caps")
  add_child(gs1)
  
  #  var gs2 = Node.new()
  #  gs2.set_script(GStreamer)
  #  gs2.image_texture = $HBoxContainer/TextureRect2.texture
  #  gs2.pipeline = "videotestsrc ! videoconvert ! video/x-raw, format=RGB ! appsink name=videosink emit-signals=true sync=false max-buffers=1 drop=true"
  #  add_child(gs2)
  print("Main ready")

func _on_GStreamer_new_caps(_node, caps):
  print("Left steamer caps: %s" % caps)
