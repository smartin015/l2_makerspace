extends Spatial

onready var PuppetPlayer = load("res://PuppetPlayer.tscn")
onready var player = $Players/Player

puppet func spawn_player(id):
  if gamestate.players.get(id) == null:
    print("unregistered player %s spawn attempted; ignoring" % id)
    return
  # Don't sppawn self; already spawned
  if gamestate.players[id] == gamestate.my_name:
    print("self-spawned; starting to send position")
    gamestate.is_initialized = true
    return

  var player = PuppetPlayer.instance()
  player.name = gamestate.players[id] # Important must be same across network
  player.set_network_master(id) # Important
  $Players.add_child(player)
  print("Created puppet %s (id %d)" % [player.name, id])

puppet func spawn_cube(origin):
  var cube = MeshInstance.new()
  cube.mesh = CubeMesh.new()
  cube.scale = Vector3(0.1,0.1,0.1)
  cube.transform.origin = origin
  $Cubes.add_child(cube)
  print("Added cube at %s" % origin)

puppet func spawn_poly(asset_id):
  $MeshStreamer.stream(asset_id)

puppet func remove_player(id):
  var node = $Players.get_node(String(id))
  if node != null:
    node.queue_free()

func _ready():
  gamestate.init()
  var err = gamestate.connect("connection_failed", self, "_on_connection_failed")
  if err != OK:
    print("error %d registering connection_failed" % err)
  err = gamestate.connect("connection_succeeded", self, "_on_connection_success")
  if err != OK:
    print("error %d registering connection_succeeded" % err)
  err = gamestate.connect("server_disconnected", self, "_on_server_disconnect")
  if err != OK:
    print("error %d registering server_disconnected" % err)
  err = player.connect("spawn_cube_request", self, "_on_spawn_cube_request")
  if err != OK:
    print("error %d registering spawn_cube_request" % err)
  err = $MeshStreamer.connect("mesh_loaded", self, "_on_mesh_loaded")
  if err != OK:
    print("error %d registering mesh_loaded" % err)
  spawn_poly("5vbJ5vildOq")

func _on_connection_success():
  gamestate.my_name = str(get_tree().get_network_unique_id())
  $Players/Player.name = gamestate.my_name
  print("Connected; player name now %s" % gamestate.my_name)
  gamestate.pre_start_game()
  
func _on_connection_failed():
  print("Connection Failed, trying again")

func _on_server_disconnect():
  print("Server Disconnected, trying to connect...")
  
func _on_spawn_cube_request():
  gamestate.rpc_id(1, "spawn_cube", player.get_node("ARVROrigin/LeftHand").transform.origin)

func _on_mesh_loaded(mi: MeshInstance):
  mi.transform.origin = Vector3(3, 1, 0)
  self.add_child(mi)

# ==================== Grab logic =====================================
onready var lgrab = $Players/Player/OQ_ARVROrigin/OQ_LeftController/Feature_StaticGrab
onready var rgrab = $Players/Player/OQ_ARVROrigin/OQ_RightController/Feature_StaticGrab
var grabStart

func _process(_delta):
  if lgrab.is_just_grabbing:
    grabStart = lgrab.grabbed_object.translation
  elif lgrab.is_grabbing:
    lgrab.grabbed_object.get_parent().set_pos(lgrab.grabbed_object.name, lgrab.delta_position + grabStart)
