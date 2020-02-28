extends HTTPRequest

const SETTINGS_URI = "https://raw.githubusercontent.com/smartin015/l2_makerspace/master/infra/vr/client.conf"
const DEFAULT_SETTINGS = {
  "server_ip": "127.0.0.1",
  "server_port": 44444,
}
const DEFAULT_CONNECT_TIMEOUT = 0.5
var settings = DEFAULT_SETTINGS

var host = NetworkedMultiplayerENet.new()
onready var player = get_node("/root/World/Players/Player")
onready var players = get_node("/root/World/Players")

var is_initialized = false # We have been registered to the server.
var default_connect_timer = null

func _ready():
  var LISTENERS = {
    "connected_to_server": [get_tree(), "_connected_ok"],
    "connection_failed": [get_tree(), "_connected_fail"],
    "server_disconnected": [get_tree(), "_server_disconnected"],
    "request_completed": [self, "_on_HTTPRequest_request_completed"],
  }
  for k in LISTENERS.keys():  
    var err = LISTENERS[k][0].connect(k, self, LISTENERS[k][1])
    if err != OK:
      print("error registering ", k, ": ", err)
  is_initialized = false
  if !vr.initialize():
    print("GDT non-VR demo mode")

  default_connect_timer = Timer.new()
  default_connect_timer.set_one_shot(true)
  default_connect_timer.connect("timeout", self, "_default_connect_timeout")
  add_child(default_connect_timer)
  connect_to_server(DEFAULT_SETTINGS.server_ip, DEFAULT_SETTINGS.server_port) 
  default_connect_timer.start(DEFAULT_CONNECT_TIMEOUT)
  
func _on_HTTPRequest_request_completed(_result, response_code, _headers, body):
  if response_code != 200:
    print("GDT settings code %d; cannot start" % response_code)
    return

  print("GDT settings code ", response_code)
  var json = JSON.parse(body.get_string_from_utf8())
  if json.error == OK:
    if typeof(json.result.get("server_ip")) != TYPE_STRING || typeof(json.result.get("server_port")) == TYPE_NIL:
      print("GDT settings invalid: %s" % json.result)
    else:
      print("Got settings: %s" % json.result)
      settings = json.result
  else:
    print("GDT settings parse err: %s " % json.error_string)
  connect_to_server(settings.server_ip, settings.server_port)

func _default_connect_timeout():
  if host.get_connection_status() != NetworkedMultiplayerPeer.CONNECTION_CONNECTED:
    host.close_connection()
    print("GDT no localhost; fetching settings")
    var _ignore = self.request(SETTINGS_URI)

func connect_to_server(ip, port):
  print("GDT conn %s:%d" % [ip, port])
  if host.get_connection_status() != host.CONNECTION_DISCONNECTED:
    host.close_connection()
  
  var err = host.create_client(ip, port)
  if err != OK:
    print("GDT conn error %s" % err)
    return

  get_tree().set_network_peer(host)

func _connected_ok():
  player.name = str(get_tree().get_network_unique_id())
  # $MeshStreamer.spawn("5vbJ5vildOq")

func _server_disconnected():
  is_initialized = false
  players.clear()
  connect_to_server(settings.server_ip, settings.server_port)

func _connected_fail():
  is_initialized = false
  get_tree().set_network_peer(null)
  connect_to_server(settings.server_ip, settings.server_port)
