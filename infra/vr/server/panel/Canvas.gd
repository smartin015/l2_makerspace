extends Spatial

const objtype = "CANVAS"
export var workspace = gamestate.DEFAULT_WORKSPACE
var currLine = {}

remote func clear():
  for l in get_children():
    l.queue_free()

remote func setup_request():
  var sender = get_tree().get_rpc_sender_id()
  var linesList = []
  for l in get_children():
    if l is Line2D:
      linesList.push_back(l.points)
  rpc_id(sender, "setup", linesList)

remote func handle_input(pressed, position):
  var sender = get_tree().get_rpc_sender_id()
  if pressed != null:
    if pressed:
      var nl = Line2D.new()
      add_child(nl)
      nl.add_point(position)
      currLine[sender] = nl
    elif currLine.get(sender) != null:
      currLine[sender] = null
  else:
    var cl = currLine.get(sender)
    if cl != null:
      cl.add_point(position)
