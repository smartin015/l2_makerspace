extends Node

onready var GStreamer = load("res://bin/gstreamer.gdns")

func _ready():
  var gs1 = Node.new()
  gs1.set_script(GStreamer)
  gs1.image_texture = $HBoxContainer/TextureRect.texture
  gs1.pipeline = "uridecodebin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm ! videoconvert ! video/x-raw, format=RGB ! appsink name=videosink emit-signals=true sync=false max-buffers=1 drop=true"
  gs1.connect("new_caps", self, "_on_GStreamer_new_caps")
  add_child(gs1)
  
  var gs2 = Node.new()
  gs2.set_script(GStreamer)
  gs2.image_texture = $HBoxContainer/TextureRect2.texture
  gs2.pipeline = "videotestsrc ! videoconvert ! video/x-raw, format=RGB ! appsink name=videosink emit-signals=true sync=false max-buffers=1 drop=true"
  add_child(gs2)
  print("Main ready")

func _on_GStreamer_new_caps(_node, caps):
  print("Left steamer caps: %s" % caps)
