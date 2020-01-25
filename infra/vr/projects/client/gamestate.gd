extends HTTPRequest

# Game port and ip
const SETTINGS_URI = "https://raw.githubusercontent.com/smartin015/l2_makerspace/master/infra/vr/client.conf"
const DEFAULT_SETTINGS = {
  "server_ip": "127.0.0.1",
  "server_port": 44444,
}
var settings = DEFAULT_SETTINGS
const POLY_API_KEY = "AIzaSyDxj27BQYsrrMFZg1MJDs17iUN1roPdifc"

# Signal to let GUI know whats up
signal connection_failed()
signal connection_succeeded()
signal server_disconnected()

var my_name = "Client"
var players = {} # Players dict stored as id:name
var is_initialized = false # We have been registered to the server.

var outbound

func _ready():
  var err = get_tree().connect("connected_to_server", self, "_connected_ok")
  if err != OK:
    print("error %s registering connected_to_server" % err)
  err = get_tree().connect("connection_failed", self, "_connected_fail")
  if err != OK:
    print("error %s registering connection_failed" % err)
  err = get_tree().connect("server_disconnected", self, "_server_disconnected")	
  if err != OK:
    print("error %s registering server_disconnected" % err)
  is_initialized = false
  
  err = self.connect("request_completed", self, "_on_HTTPRequest_request_completed")
  if err != OK:
    print("error %s self-registering for http requests" % err)
    
  outbound = SETTINGS_URI
  print("Fetching remote settings...")
  self.request(SETTINGS_URI)
  #connect_to_server()

func _on_HTTPRequest_request_completed(result, response_code, headers, body):
  if outbound == SETTINGS_URI:
    if response_code != 200:
      print("Got code %d requesting online settings, using defaults instead" % response_code)
    else:
      var json = JSON.parse(body.get_string_from_utf8())
      if json.error == OK:
        if typeof(json.result.get("server_ip")) != TYPE_STRING || typeof(json.result.get("server_port")) == TYPE_NIL:
          print("Missing/wrong type settings in result %s" % json.result)
        else:
          print("Got settings from remote: %s" % json.result)
          settings = json.result
      else:
        print("JSON parse error: %s " % json.error_string)
    connect_to_server() # Try to connect
    # asset_search("piano")
  elif outbound == "poly_list":
    var json = JSON.parse(body.get_string_from_utf8())
    print(json.result.assets[0])
    for fmt in json.result.assets[0].formats:
      if fmt.formatType == "GLTF2":
        print("%s %s" % [fmt.formatType, fmt.resources])
        outbound = "poly_download"
        self.request(fmt.resources[0].url)
  elif outbound == "poly_download":
    print(response_code)
    var dat_out = File.new()
    dat_out.open('user://thing.bin', File.WRITE)
    dat_out.store_buffer(body)
    dat_out.close()
    var mesh=load('user://thing.bin')
    print(mesh)
    if mesh != null:
      get_node('/root/World').add_child(mesh.instance())


func asset_search(params):
  outbound = "poly_list"
  self.request("https://poly.googleapis.com/v1/assets?keywords=%s&key=%s" % [params, POLY_API_KEY])

func connect_to_server():
  print("Attempting to connect to server %s port %d" % [settings.server_ip, settings.server_port])
  var host = NetworkedMultiplayerENet.new()
  host.create_client(settings.server_ip, settings.server_port)
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
  print("registered player %s: %s" % [id, new_player_data])

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
  
func remote_log(text):
  rpc_id(1, "remote_log", text)

remote func recv_remote_log(text):
  print(text)
