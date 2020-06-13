extends Node

const PORT = 44444
const MAX_PLAYERS = 12
const DEFAULT_WORKSPACE = "0"
onready var actors = get_node("/root/World/Actors")
onready var players = get_node("/root/World/Players")
onready var tools = get_node("/root/World/Tools")
onready var PathEnt = load("res://PathEnt.gd")
onready var workspaces = _init_workspaces()

# Shapes for CAD
enum SHAPE {PENCIL, LINE, RECTANGLE, CIRCLE, DRAG}

func _ready():
  var err = get_tree().connect("network_peer_connected", self, "_peer_connected")
  if err != OK:
    print("network_peer_connected err: ", err)
  err = get_tree().connect("network_peer_disconnected", self,"_peer_disconnected")	
  if err != OK:
    print("network_peer_disconnected err: ", err)
  
  var host = NetworkedMultiplayerENet.new()
  host.create_server(PORT, MAX_PLAYERS)
  get_tree().set_network_peer(host)
  print("GDT listen port ", PORT)

remote func set_workspace(ws):
  var sender = get_tree().get_rpc_sender_id()
  var p = players.find_node(str(sender), true, false)
  if p == null:
    print("Could not set workspace for %s to %s: player not found" % [sender, ws])
  p.rpc("set_workspace", ws)
  _populate_workspace_for_player(sender, ws)
  
func _get_visible_workspaces():
  # TODO visibility rules
  var visible_ws = PoolStringArray()
  for w in workspaces:
    visible_ws.push_back(w.name)
  return visible_ws
  
func _broadcast_visible_workspaces():
  for p in players.get_children():
    rpc_id(p.get_network_master(), "set_visible_workspaces", _get_visible_workspaces())
  
func _peer_connected(id):
  print("GDT(%s) connected; pushing %d players %d actors" % [id, len(players.get_children()), len(actors.get_children())])
  
  # Send a list of user-visible workspaces
  rpc_id(id, "set_visible_workspaces", _get_visible_workspaces())
  
  # User begins in the default workspace
  _populate_workspace_for_player(id, DEFAULT_WORKSPACE)
  
  # Spawn other players for new peer
  for p in players.get_children():
    players.rpc_id(id, "spawn", p.get_network_master(), p.transform.origin, p.workspace)
  
  # Spawn new peer for all players
  players.rpc("spawn", id, Vector3.ZERO, DEFAULT_WORKSPACE)
  
  # Let user know about ROS peers
  ROSBridge.send_ros_peers()  
  
func _populate_workspace_for_player(id, ws):
  # Spawn all currently active dynamic elements on new client
  # Note that (puppet) players are never removed, so aren't
  # re-populated here.
  
  for a in actors.get_children():
    if a.workspace != ws:
      continue
    actors.rpc_id(id, "spawn", a.name, a.objtype, a.config, a.transform, a.peer_id)
  
  for t in tools.get_children():
    if t.workspace != ws:
      continue
    if !t.get("objtype"):
      # TODO some tools not yet set up for workspaces
      continue
    tools.rpc_id(id, "spawn", t.name, t.objtype, t.transform)

func _peer_disconnected(id):
  players.rpc("remove", id) # Remove peer everywhere
  print("GDT(%s) disconnected" % id)
  
remote func recv_shout(text: String):
  var sender = get_tree().get_rpc_sender_id()
  print("%s: %s" % [sender, text])

remote func request_new_ws():
  var ws = PathEnt.new()
  ws.name = "new space"
  ws.perm = 0x777
  workspaces.push_back(ws)
  _broadcast_visible_workspaces()
  rpc_id(get_tree().get_rpc_sender_id(), "new_ws", ws.name)

remote func edit_workspace(ws, fields):
  for w in workspaces:
    if w.name == ws:
      w.name = fields["name"]
      _broadcast_visible_workspaces()
      return
    
func _init_workspaces():
  var wss = []
  for i in range(3):
    var ws = PathEnt.new()
    ws.name = str(i)
    ws.perm = 0x777
    wss.push_back(ws)
  return wss
