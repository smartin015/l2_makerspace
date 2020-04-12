extends Node

const PORT = 44444
const MAX_PLAYERS = 12
onready var actors = get_node("/root/World/Actors")
onready var players = get_node("/root/World/Players")

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

func _peer_connected(id):
  print("GDT(%s) connected; pushing %d players %d actors" % [id, len(players.get_children()), len(actors.get_children())])
  
  # Spawn all currently active dynamic elements for new r
  for p in players.get_children():
    players.rpc_id(id, "spawn", p.get_network_master())
  for a in actors.get_children():
    actors.rpc_id(id, "spawn", a.name, a.objtype, a.config, a.transform, a.peer_id)
  
  # Let user know about ROS peers
  ROSBridge.send_ros_peers()
  
  players.rpc("spawn", id) # Spawn new peer everywhere

func _peer_disconnected(id):
  players.rpc("remove", id) # Remove peer everywhere
  print("GDT(%s) disconnected" % id)
