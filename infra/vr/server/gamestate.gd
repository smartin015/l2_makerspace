extends Node

const PORT = 44444
const MAX_PLAYERS = 12
onready var actors = get_node("/root/World/Actors")
onready var players = get_node("/root/World/Players")
onready var tools = get_node("/root/World/Tools")

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

func _peer_connected(id):
  print("GDT(%s) connected; pushing %d players %d actors" % [id, len(players.get_children()), len(actors.get_children())])
  
  # Send a list of user-visible workspaces
  workspace.broadcast_visible(id)
  
  # User begins in the default workspace
  _populate_workspace_for_player(id, workspace.DEFAULT)
  
  # Spawn other players for new peer
  for p in players.get_children():
    players.rpc_id(id, "spawn", p.get_network_master(), p.transform.origin, p.ws)
  
  # Spawn new peer for all players
  players.rpc("spawn", id, Vector3.ZERO, workspace.DEFAULT)
  
  # Let user know about ROS peers
  ROSBridge.send_ros_peers()  
  
func _populate_workspace_for_player(id, ws):
  # Spawn all currently active dynamic elements on new client
  # Note that (puppet) players are never removed, so aren't
  # re-populated here.
  
  for a in actors.get_children():
    if a.ws != ws:
      continue
    actors.rpc_id(id, "spawn", a.name, a.objtype, a.config, a.transform, a.peer_id)
  
  for t in tools.get_children():
    if t.ws != ws:
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
