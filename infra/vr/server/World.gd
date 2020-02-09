extends Spatial

onready var PuppetPlayer = load("res://PuppetPlayer.tscn")

puppetsync func spawn_player(id, tf):
  var player = PuppetPlayer.instance()
  player.name = gamestate.players[id] # Important
  player.set_network_master(id) # Important
  player.puppet_transform = tf
  $Players.add_child(player)
  print("Initialized player, node %s id %d" % [player.name, id])

puppetsync func spawn_cube(origin):
  var cube = MeshInstance.new()
  cube.mesh = CubeMesh.new()
  cube.scale = Vector3(0.1,0.1,0.1)
  cube.transform.origin = origin
  $Cubes.add_child(cube)
  print("Added cube at %s" % origin)

puppetsync func remove_player(id):
  $Players.get_node(gamestate.players[id]).queue_free()
