extends HTTPRequest

const SETTINGS_URI = "https://raw.githubusercontent.com/smartin015/l2_makerspace/master/infra/vr/client.conf"
const DEFAULT_SETTINGS = {
  "server_ip": "127.0.0.1",
  "server_port": 44444,
}
const DEFAULT_CONNECT_TIMEOUT = 0.5
const DEFAULT_WORKSPACE = "0"
var workspaces = []
var new_ws_cb = null
var settings = DEFAULT_SETTINGS

var host = NetworkedMultiplayerENet.new()
onready var player = get_node("/root/World/Players/Player")
onready var players = get_node("/root/World/Players")
onready var actors = get_node("/root/World/Actors")
onready var tools = get_node("/root/World/Tools")

# Shapes for CAD
enum SHAPE {PENCIL, LINE, RECTANGLE, CIRCLE, DRAG}

var is_initialized = false # We have been registered to the server.
var default_connect_timer = null

func _ready():
  if !vr.initialize():
    print("GDT non-VR demo mode")
  
  if get_tree().get_current_scene().get_name() != "World":
    print("Not in main scene; skipping connections")
    return
  
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

func _server_disconnected():
  is_initialized = false
  players.clear()
  connect_to_server(settings.server_ip, settings.server_port)

func _connected_fail():
  is_initialized = false
  get_tree().set_network_peer(null)
  connect_to_server(settings.server_ip, settings.server_port)

func set_workspace(ws):
  if ws == player.workspace:
    return # nothing to do

  # Remove all the stuff from the prior workspace
  for a in actors.get_children():
    a.queue_free()
  for t in tools.get_children():
    t.queue_free()
    
  # It's up to the server to fulfill the workspace change,
  # after which it calls set_workspace() on the player node
  rpc_id(1, "set_workspace", ws)
  
remote func set_visible_workspaces(visible_ws: PoolStringArray):
  workspaces = visible_ws
  
func request_new_ws(obj, method):
  new_ws_cb = [obj, method]
  rpc_id(1, "request_new_ws")
  
remote func new_ws(ws):
  new_ws_cb[0].call(new_ws_cb[1], ws)
  new_ws_cb = null
  
func edit_workspace(ws, fields):
  rpc_id(1, "edit_workspace", ws, fields)
  
func shout(text: String):
  rpc("recv_shout", text)
  
remote func recv_shout(text: String):
  var sender = get_tree().get_rpc_sender_id()
  print("%s: %s" % [sender, text])
