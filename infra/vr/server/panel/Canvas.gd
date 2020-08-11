extends Spatial

onready var L2Shape = load("res://panel/Shape.tscn")
const objtype = "CANVAS"
export var ws = workspace.DEFAULT
var currShape = {}

remote func clear():
  for c in get_children():
    c.queue_free()

remotesync func undo():
  # Undo hides the last unhidden shape
  var cs = get_children()
  cs.invert()
  for c in cs:
    if c.visible:
      c.visible = false
      return

remotesync func redo():
  # Redo unhides the last hidden shape
  var cs = get_children()
  cs.invert()
  for c in cs:
    if not c.visible:
      c.visible = true
      return

remote func save():
  print("Beginning save")
  var shapes = ""
  for c in get_children():
    if not c.has_method("to_svg"):
      continue
    shapes += "\n" + c.to_svg()
    
  var svg = ("<?xml version=\"1.0\" standalone=\"yes\"?>"
      + "<svg width=\"1500\" height=\"600\" xmlns=\"http://www.w3.org/2000/svg\">"
      + shapes
      + "\n</svg>")
  
  var path = "canvas_%s_%s.svg" % [name, OS.get_unix_time()]
  ROSBridge.publish("PutFile", "l2_msgs/msg/L2File", {
    "path": path,
    "data": svg,
  }, path)
  
  var sender = get_tree().get_rpc_sender_id()
  # TODO notify write completion
  rpc_id(sender, "on_save", OK, "writing file") 

func get_state():
  var shapesList = []
  for s in get_children():
    if s.has_method('start_shape') and s.visible:
      shapesList.push_back([s.shapeType, s.color, s.points])
  return {"type": objtype, "shapes": shapesList}

remote func setup_request():
  rpc_id(get_tree().get_rpc_sender_id(), "setup", get_state()["shapes"])

remote func handle_input(pressed, position, shapeType, col):
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
      n.start_shape(shapeType, position, col)
      add_child(n)
      currShape[sender] = n
    elif currShape.get(sender) != null:
      currShape[sender] = null
  else:
    var cl = currShape.get(sender)
    if cl != null:
      cl.handle_point(position)
