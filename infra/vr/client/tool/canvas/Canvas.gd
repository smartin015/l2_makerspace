extends Spatial

var currShape = {}
var cui
var nextShapeType = gamestate.SHAPE.LINE
onready var audio = $AudioStreamPlayer2D
onready var L2Shape = load("res://tool/canvas/Shape.tscn")

remote func set_tf(tf):
  transform = tf

remote func set_ws(ws):
  self.ws = ws

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

remotesync func undo():
  # Undo hides the last unhidden shape
  if cui == null:
    return
  var cs = cui.get_children()
  cs.invert()
  for c in cs:
    if c.has_method('start_shape') and c.visible:
      c.visible = false
      c.pickable = false
      return

remotesync func handle_input(pressed, position, shapeType):
  if cui == null:
    return
  
  # Remove any undo (non-visible) shapes
  var cs = cui.get_children()
  cs.invert()
  for c in cs:
    if not c.has_method('start_shape'):
      continue
    if c.visible:
      break
    c.queue_free()
  
  match shapeType:
    gamestate.SHAPE.DRAG:
      # Input is already forwarded to canvas UI (Mouse > Filter == "Pass")
      pass
    _:
      pass
  
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

func _on_CanvasUIContainer_save():
  # TODO save
  print("Save")
  audio.play()

func _on_CanvasUIContainer_set_shape(shape):
  # Shape is only selected locally
  if nextShapeType == gamestate.SHAPE.DRAG && shape != nextShapeType:
    for c in cui.get_children():
      if c.visible:
        c.pickable = false
  nextShapeType = shape
  if shape == gamestate.SHAPE.DRAG:
    for c in cui.get_children():
      if c.visible:
        c.pickable = true
  print("Set next shape type to %s" % gamestate.SHAPE.keys()[shape])

func _on_AudioStreamPlayer2D_finished():
  print("Audio finished")

func _on_CanvasUIContainer_undo():
  rpc("undo")
