extends GraphNode

var params

func _ready():
  set_slot(0, true, 0, Color(1,1,1,1), true, 0, Color(0,1,0,1), null, null)
  print("Slots: %s %s" % [get_connection_input_count(), get_connection_output_count()])

remote func set_offset(offs):
  offset = offs

remote func remove():
  queue_free()

remote func set_params(p):
  params = p
