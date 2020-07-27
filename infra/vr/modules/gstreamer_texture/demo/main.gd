extends Node

func _ready():
  print("Main ready")

func _on_GStreamer_new_caps(node, caps):
  print("Left steamer caps: %s" % caps)
