extends GraphNode

onready var tex = load("res://WhiteDot.svg")

func _ready():
  set_slot(0, true, 0, Color(1,1,1,1), true, 0, Color(0,1,0,1), tex, tex)

func _on_SequenceItem_close_request():
  rpc("remove")
  
remotesync func remove():
  queue_free()

func _on_SequenceItem_dragged(from, to):
  rpc("set_offset", to)

remotesync func set_offset(offs):
  self.offset = offs
