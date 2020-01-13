extends Node

# Game port and ip
const ip = "127.0.0.1"
const DEFAULT_PORT = 44444

# Signal to let GUI know whats up
signal connection_failed()
signal connection_succeeded()
signal server_disconnected()

var my_name = "Client"
var players = {} # Players dict stored as id:name
var is_initialized = false # We have been registered to the server.

func _ready():
	var err = get_tree().connect("connected_to_server", self, "_connected_ok")
	if err != OK:
		print("error %d registering connected_to_server")
	err = get_tree().connect("connection_failed", self, "_connected_fail")
	if err != OK:
		print("error %d registering connection_failed")
	err = get_tree().connect("server_disconnected", self, "_server_disconnected")	
	if err != OK:
		print("error %d registering server_disconnected")
	print("Attempting to connect to server %s port %d" % [ip, DEFAULT_PORT])
	is_initialized = false
	connect_to_server() # Try to connect right away

func connect_to_server():
	var host = NetworkedMultiplayerENet.new()
	host.create_client(ip, DEFAULT_PORT)
	get_tree().set_network_peer(host)

# Callback from SceneTree, called when connect to server
func _connected_ok():
	emit_signal("connection_succeeded")

# Callback from SceneTree, called when server disconnect
func _server_disconnected():
	is_initialized = false
	players.clear()
	emit_signal("server_disconnected")
	# Try to connect again
	connect_to_server()

# Callback from SceneTree, called when unabled to connect to server
func _connected_fail():
	is_initialized = false
	get_tree().set_network_peer(null) # Remove peer
	emit_signal("connection_failed")
	# Try to connect again
	connect_to_server()

puppet func register_player(id, new_player_data):
	players[id] = new_player_data

puppet func unregister_player(id):
	players.erase(id)
# Returns list of player names
func get_player_list():
	return players.values()

puppet func pre_start_game():
	# Register ourselves with the server
	rpc_id(1, "register_player", my_name)
	# Tell Server we ready to roll
	rpc_id(1, "populate_world")
