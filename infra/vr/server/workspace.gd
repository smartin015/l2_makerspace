extends Node
const DEFAULT = "0"
onready var PathEnt = load("res://PathEnt.gd")
onready var workspaces = _init_workspaces()

const DEFAULT_WS_ROOMS = ["csg", "concept", "hh", "spaceship"]

func _get_visible():
  # TODO visibility rules
  var visible_ws = {}
  for w in workspaces:
    visible_ws[w.name] = w.fields
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
  ws.fields["room"] = DEFAULT_WS_ROOMS[0]
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
  for i in range(len(DEFAULT_WS_ROOMS)):
    var ws = PathEnt.new()
    ws.name = "Workspace " + str(i)
    ws.perm = 0x777
    ws.fields["room"] = DEFAULT_WS_ROOMS[i]
    wss.push_back(ws)
  return wss

remote func snapshot(name):
  var sender = get_tree().get_rpc_sender_id()
  print("TODO snapshot ws %s" % name)
  var ws
  for w in workspaces:
    if w.name == name:
      ws = w.get_state()
      
  if ws == null:
    rpc_id(sender, "on_snapshot", name, ERR_CANT_RESOLVE)
    return
  
  var tools = []
  for t in gamestate.tools.get_children():
    if t.ws == name:
      tools.push_back(t.get_state())
  
  print({"ws": ws, "tools": tools})
  
  rpc_id(sender, "on_snapshot", name, OK)
  
