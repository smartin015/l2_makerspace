extends Spatial

onready var PuppetPlayer = load("res://PuppetPlayer.tscn")

puppet func spawn_player(id):
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

puppet func remove_player(id):
	$Players.get_node(String(id)).queue_free()

func _ready():
	var err = gamestate.connect("connection_failed", self, "_on_connection_failed")
	if err != OK:
		print("error %d registering connection_failed")
	err = gamestate.connect("connection_succeeded", self, "_on_connection_success")
	if err != OK:
		print("error %d registering connection_succeeded")
	err = gamestate.connect("server_disconnected", self, "_on_server_disconnect")
	if err != OK:
		print("error %d registering server_disconnected")
	# err = $Players/Player.connect("playerprocess", self, "_on_player_process")
	
func _on_connection_success():
	randomize() # Reset seed
	
	print("Connected successfully!")
	# TODO Something a bit more permanent / useful. 
	gamestate.my_name = str(int(rand_range(0,10000000)))
	$Players/Player.name = gamestate.my_name
	print("Set player name to %s" % gamestate.my_name)
	gamestate.pre_start_game()
	
func _on_connection_failed():
	print("Connection Failed, trying again")

func _on_server_disconnect():
	print("Server Disconnected, trying to connect...")
