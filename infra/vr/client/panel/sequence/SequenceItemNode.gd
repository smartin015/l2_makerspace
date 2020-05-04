extends GraphNode

onready var tex = load("res://WhiteDot.svg")

func _ready():
  print(tex.get_size())
  set_slot(0, true, 0, Color(1,1,1,1), true, 0, Color(0,1,0,1), tex, tex)


# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#  pass

func _on_SequenceItem_close_request():
  queue_free()
