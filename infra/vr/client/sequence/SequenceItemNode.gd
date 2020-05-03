extends GraphNode

func _ready():
  set_slot(0, true, 0, Color(1,1,1,1), true, 0, Color(0,1,0,1))


# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#  pass

func _on_SequenceItem_close_request():
  queue_free()
