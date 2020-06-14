extends Node
const DEFAULT = "0"
onready var PathEnt = load("res://PathEnt.gd")
onready var workspaces = _init_workspaces()

func _get_visible():
  # TODO visibility rules
  var visible_ws = PoolStringArray()
  for w in workspaces:
    visible_ws.push_back(w.name)
  return visible_ws
  
func broadcast_visible(id = null):
  if id != null:
    rpc_id(id, "set_visible", _get_visible())
    return
  for p in gamestate.players.get_children():
    rpc_id(p.get_network_master(), "set_visible", _get_visible())
  
remote func request():
  var ws = PathEnt.new()
  ws.name = "new space"
  ws.perm = 0x777
  workspaces.push_back(ws)
  broadcast_visible()
  rpc_id(get_tree().get_rpc_sender_id(), "on_new", ws.name)

remote func edit(ws, fields):
  for w in workspaces:
    if w.name == ws:
      w.name = fields["name"]
      broadcast_visible()
      return
    
func _init_workspaces():
  var wss = []
  for i in range(3):
    var ws = PathEnt.new()
    ws.name = str(i)
    ws.perm = 0x777
    wss.push_back(ws)
  return wss
