extends Node

var currLine = {}

remotesync func handle_input(event):
  var sender = get_tree().get_rpc_sender_id()
  if event is InputEventMouseButton:
    if event.pressed:
      var nl = Line2D.new()
      add_child(nl)
      nl.add_point(event.position)
      currLine[sender] = nl
    elif currLine.get(sender) != null:
      currLine[sender] = null
  elif event is InputEventMouseMotion:
    var cl = currLine.get(sender)
    if cl != null:
      cl.add_point(event.position)
