extends Spatial

onready var PuppetPlayer = load("res://PuppetPlayer.tscn")

puppetsync func spawn_player(id):
  var player = PuppetPlayer.instance()
  player.name = gamestate.players[id] # Important
  player.set_network_master(id) # Important
  $Players.add_child(player)
  print("Initialized player, node %s id %d" % [player.name, id])

puppetsync func remove_player(id):
  $Players.get_node(gamestate.players[id]).queue_free()
