extends HTTPRequest

const SETTINGS_URI = "https://raw.githubusercontent.com/smartin015/l2_makerspace/master/infra/vr/client.conf"
const DEFAULT_SETTINGS = {
  "server_ip": "127.0.0.1",
  "server_port": 44444,
}

const DEFAULT_CONNECT_TIMEOUT = 0.5
var settings = DEFAULT_SETTINGS
var config = null

var host = NetworkedMultiplayerENet.new()
onready var room = get_node("/root/World/Room")
onready var player = get_node("/root/World/Players/Player")
onready var players = get_node("/root/World/Players")
onready var actors = get_node("/root/World/Actors")
onready var tools = get_node("/root/World/Tools")
onready var nfloor = get_node("/root/World/NavFloor")
onready var L2Control = load("res://tool/menu/L2Control.tscn")
onready var Config = load("res://config.gd")

# Shapes for CAD
enum SHAPE {PENCIL, LINE, RECTANGLE, CIRCLE, DRAG}

var is_initialized = false # We have been registered to the server.
var default_connect_timer = null

func _ready():
  config = Config.load_user_config()
  Config.save_user_config(config)
  
  if !vr.initialize():
    print("GDT non-VR demo mode")
  
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
  if player != null:
    player.name = str(get_tree().get_network_unique_id())
  rpc_id(1, "request_init", gamestate.config["alias"])

func _server_disconnected():
  is_initialized = false
  if players != null:
    players.clear()
  connect_to_server(settings.server_ip, settings.server_port)

func _connected_fail():
  is_initialized = false
  get_tree().set_network_peer(null)
  connect_to_server(settings.server_ip, settings.server_port)

func set_workspace(ws):
  if ws == player.ws:
    return # nothing to do

  # Remove all the stuff from the prior workspace
  for a in actors.get_children():
    a.queue_free()
  for t in tools.get_children():
    t.queue_free()
    
  # Update the room theme for this workspace (from ws fields)
  var fields = workspace.ws_fields.get(ws)
  if fields != null and fields.has("room"):
    if room != null:
      room.queue_free()
      room = null
    
    var res = load("res://rooms/" + fields["room"].replace("/", "").replace(".", "") + "/room.tscn")
    if res == null:
      return
    room = res.instance()
    room.name = "Room"
    get_node("/root/World").add_child(room)
    print("New room scene added: %s" % [fields["room"]])
  else:
    print("Couldn't fetch room field for workspace %s" % [ws])
    
  # It's up to the server to fulfill the workspace change,
  # after which it calls set_workspace() on the player node
  rpc_id(1, "set_workspace", ws)

func handle_selection_change():
  # When user selects something, update the NavFloor
  # so we can reposition
  var s = get_tree().get_nodes_in_group("selection")
  if len(s) > 0:
    nfloor.from_pos = s[-1].global_transform.origin

func move_selection(dest):
  var s = get_tree().get_nodes_in_group("selection")
  if len(s) > 0:
    s[-1].handle_drag(dest)
    nfloor.from_pos = null
  
func ws_selection(ws):
  var s = get_tree().get_nodes_in_group("selection")
  if len(s) > 0:
    s[-1].handle_ws(ws)
    nfloor.from_pos = null
  
func delete_selection():
  var s = get_tree().get_nodes_in_group("selection")
  if len(s) > 0:
    s[-1].handle_delete()
    nfloor.from_pos = null
  
func shout(text: String):
  rpc("recv_shout", text)
  
remote func recv_shout(text: String):
  var sender = get_tree().get_rpc_sender_id()
  print("%s: %s" % [sender, text])

var menu = null
func toggle_menu(tf):
  if menu == null:
    menu = L2Control.instance()
    menu.global_transform = tf
    tools.add_child(menu)
  else:
    menu.queue_free()

func set_social(next):
  for k in next.keys():
    match k:
      "alias":
        config["alias"] = next[k]
        player.rset("alias", next[k])
      "color":
        config["color"] = next[k]
        player.rset("color", next[k])
        
  Config.save_user_config(config)
