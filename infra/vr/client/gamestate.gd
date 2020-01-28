extends HTTPRequest

# Game port and ip
const SETTINGS_URI = "https://raw.githubusercontent.com/smartin015/l2_makerspace/master/infra/vr/client.conf"
const DEFAULT_SETTINGS = {
  "server_ip": "127.0.0.1",
  "server_port": 44444,
}
var settings = DEFAULT_SETTINGS
var host = NetworkedMultiplayerENet.new()
const POLY_API_KEY = "AIzaSyDxj27BQYsrrMFZg1MJDs17iUN1roPdifc"

# Signal to let GUI know whats up
signal connection_failed()
signal connection_succeeded()
signal server_disconnected()

var my_name = "Client"
var players = {} # Players dict stored as id:name
var is_initialized = false # We have been registered to the server.
var default_connect_timer = null

var outbound

var LISTENERS = {
  "connected_to_server": "_connected_ok",
  "connection_failed": "_connected_fail",
  "server_disconnected": "_server_disconnected",
  "request_completed": "_on_HTTPRequest_request_completed",
 }

func _ready():
  for k in LISTENERS.keys():  
    var err = get_tree().connect(k, self, LISTENERS[k])
    if err != OK:
      print("error registering ", k, ": ", err)
  is_initialized = false
    
func init():
  outbound = SETTINGS_URI
  # Try briefly connecting to localhost first
  default_connect_timer = Timer.new()
  default_connect_timer.set_one_shot(true)
  default_connect_timer.connect("timeout", self, "_default_connect_timeout")
  add_child(default_connect_timer)
  
  connect_to_server(DEFAULT_SETTINGS.server_ip, DEFAULT_SETTINGS.server_port) # Try to connect
  default_connect_timer.start(0.5)
  
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
    connect_to_server(settings.server_ip, settings.server_port)
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

func _default_connect_timeout():
  if host.get_connection_status() != NetworkedMultiplayerPeer.CONNECTION_CONNECTED:
    host.close_connection()
    print("timed out connecting to localhost; fetching remote settings...")
    self.request(SETTINGS_URI)
  else: 
    print("localhost is connected")

func asset_search(params):
  outbound = "poly_list"
  self.request("https://poly.googleapis.com/v1/assets?keywords=%s&key=%s" % [params, POLY_API_KEY])

func connect_to_server(ip, port):
  print("Attempting to connect to server %s port %d" % [ip, port])
  
  # Disconnect if already connected (no-op if this is first connection)  
  host.close_connection()
  var err = host.create_client(ip, port)
  if err != OK:
    print("Error %s connecting to server" % err)
    return

  get_tree().set_network_peer(host)

# Callback from SceneTree, called when connect to server
func _connected_ok():
  emit_signal("connection_succeeded")

# Callback from SceneTree, called when server disconnect
func _server_disconnected():
  is_initialized = false
  players.clear()
  emit_signal("server_disconnected")
  connect_to_server(settings.server_ip, settings.server_port)

# Callback from SceneTree, called when unabled to connect to server
func _connected_fail():
  is_initialized = false
  get_tree().set_network_peer(null)
  emit_signal("connection_failed")
  connect_to_server(settings.server_ip, settings.server_port)

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
