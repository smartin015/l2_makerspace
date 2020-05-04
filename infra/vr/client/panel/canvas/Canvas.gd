extends Spatial

var currLine = {}
var cui

func _ready():
  cui = find_node("CanvasUI", true, false)
  # Always server-owned
  set_network_master(0)

remotesync func handle_input(event):
  if cui == null:
    return
  var sender = get_tree().get_rpc_sender_id()
  if event is InputEventMouseButton:
    if event.pressed:
      var nl = Line2D.new()
      cui.add_child(nl)
      nl.add_point(event.position)
      currLine[sender] = nl
    elif currLine.get(sender) != null:
      currLine[sender] = null
  elif event is InputEventMouseMotion:
    var cl = currLine.get(sender)
    if cl != null:
      cl.add_point(event.position)

func _on_CanvasUI_gui_input(event):
  rpc("handle_input", event)
