extends Spatial

onready var L2Shape = load("res://panel/Shape.tscn")
const objtype = "CANVAS"
export var workspace = gamestate.DEFAULT_WORKSPACE
var currShape = {}

remote func clear():
  for l in get_children():
    l.queue_free()

remotesync func undo():
  # Undo hides the last unhidden shape
  var cs = get_children()
  cs.invert()
  for c in cs:
    if c.visible:
      c.visible = false
      return

remote func setup_request():
  var sender = get_tree().get_rpc_sender_id()
  var shapesList = []
  
  for s in get_children():
    if s.has_method('start_shape') and s.visible:
      shapesList.push_back([s.shapeType, s.points])
  rpc_id(sender, "setup", shapesList)

remote func handle_input(pressed, position, shapeType):
  # Remove any undo (non-visible) shapes
  var cs = get_children()
  cs.invert()
  for c in cs:
    if not c.has_method('start_shape'):
      continue
    if c.visible:
      break
    c.queue_free()
    
  if shapeType == gamestate.SHAPE.DRAG:
    return
    
  var sender = get_tree().get_rpc_sender_id()
  if pressed != null:
    if pressed:
      var n = L2Shape.instance()
      n.start_shape(shapeType, position)
      add_child(n)
      currShape[sender] = n
    elif currShape.get(sender) != null:
      currShape[sender] = null
  else:
    var cl = currShape.get(sender)
    if cl != null:
      cl.handle_point(position)
