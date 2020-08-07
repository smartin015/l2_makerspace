extends GraphNode

onready var status = $Status

func _ready():
  set_slot(0, true, 0, Color(1,1,1,1), true, 0, Color(0,1,0,1), null, null)

func _on_SequenceItem_close_request():
  rpc("remove")
  
remotesync func remove():
  queue_free()

func _on_SequenceItem_dragged(_from, to):
  rpc("set_offset", to)

remotesync func set_offset(offs):
  self.offset = offs

func set_status_label(text):
  status.text = text
