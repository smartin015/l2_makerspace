extends Spatial

const objtype = "SEQUENCE"
puppet var ws = workspace.DEFAULT
onready var nodes = $OQ_UI2DCanvas/Viewport/SequenceUI/MarginContainer/VBoxContainer/Spacer/Nodes

remote func set_tf(tf):
  transform = tf

remote func set_ws(ws):
  self.ws = ws

func _pack_state():
  # Pack all node positions, node data/names, and 
  # connections between nodes
  var ns = []
  for n in nodes.get_children():
    if n.get("offset") != null:
      ns.push_back([n.title, n.name, n.offset])
  return {
    "nodes": ns,
    "connection_list": nodes.get_connection_list(),
  }
  
remote func setup_request():
  rpc_id(get_tree().get_rpc_sender_id(), "setup", _pack_state())
