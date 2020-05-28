extends Node

puppet var workspace = gamestate.DEFAULT_WORKSPACE
var currLine = {}

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
