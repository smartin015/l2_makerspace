extends Spatial

var currShape = {}
var cui
var nextShapeType = gamestate.SHAPE.LINE
onready var audio = $AudioStreamPlayer2D
onready var L2Shape = load("res://panel/canvas/Shape.tscn")

func _ready():
  cui = find_node("CanvasUI", true, false)
  cui.connect("gui_input", self, "_on_CanvasUI_gui_input")
  # Always server-owned
  set_network_master(0)

remote func setup(shapesList: Array):
  for s in shapesList:
    var n = L2Shape.instance()
    n.start_shape(s[0], s[1][0])
    for i in range(1, len(s[1])):
      n.handle_point(s[1][i])
    cui.add_child(n)

remotesync func clear():
  if cui == null:
    return
  for c in cui.get_children():
    if c.has_method('start_shape'):
      c.queue_free()

remotesync func handle_input(pressed, position, shapeType):
  if cui == null:
    return
  var sender = get_tree().get_rpc_sender_id()
  if pressed != null:
    if pressed:
      var n = L2Shape.instance()
      cui.add_child(n)
      n.start_shape(shapeType, position)
      currShape[sender] = n
    elif currShape.get(sender) != null:
      currShape[sender] = null
  else:
    var cl = currShape.get(sender)
    if cl != null:
      cl.handle_point(position)

func _on_CanvasUI_gui_input(event):
  # Only pass necessary event params to avoid remote
  # code execution from type inference
  # https://github.com/godotengine/godot/issues/28734
  rpc("handle_input", 
    event.get('pressed'), 
    event.get('position'), 
    nextShapeType)


func _on_CanvasUIContainer_clear():
  rpc("clear")
  clear()

func _on_CanvasUIContainer_save():
  # TODO save
  print("Save")
  audio.play()

func _on_CanvasUIContainer_set_shape(shape):
  # Shape is only selected locally
  nextShapeType = shape
  print("Set next shape type to %s" % gamestate.SHAPE.keys()[shape])

func _on_AudioStreamPlayer2D_finished():
  print("Audio finished")


