# Implements a websocket server for connecting to L2 makerspace ros2-web-bridge nodes.
# For rosbridge protocol details, see
# # https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md
extends Node

const ALIVE_INTERVAL = 5.0
const WS_PORT = 4243
const FILE_SIZE_LIMIT = 1024*1024

var connection_msgs = [
  { 
    "op": "advertise_service",
    "service": "PushObject3D",
    "type": "l2_msgs/PushObject3D"
  },
  { 
    "op": "advertise",
    "topic": "/vr/Pendants/Pendant",
    "type": "geometry_msgs/Vector3"
  },
  {
    "op": "advertise",
    "topic": "/vr/alive",
    "type": "std_msgs/Bool"
  },
]

var _server = WebSocketServer.new()
var _timer
var peers = {}

func _ready():
  _server.connect("client_connected", self, "_connected")
  _server.connect("client_disconnected", self, "_disconnected")
  _server.connect("client_close_request", self, "_close_request")
  _server.connect("data_received", self, "_on_data")
  if(_server.listen(WS_PORT) != OK):
    print("An error occurred listening on port", WS_PORT)
  else:
    print("Listening for bridge connections on port ", WS_PORT)
  _timer = Timer.new()
  _timer.wait_time = ALIVE_INTERVAL
  _timer.connect("timeout",self,"_on_timeout") 
  add_child(_timer)
  _timer.start()

func _setup_ros_listeners(id):
  print("Setting up ros listeners")
  var peer = _server.get_peer(id)
  for msg in connection_msgs:
    peer.put_packet(JSON.print(msg).to_utf8())

func _publish(topic, msg):
  var packet = JSON.print({
      "op": "publish",
      "topic": topic,
      "msg": msg
     })
  for p in peers.keys():
    if !peers[p]:
      continue
    _server.get_peer(p).put_packet(packet.to_utf8())

var delaytmr
var peerid
func _connected(id, proto):
  print("Client %d connected with protocol: %s" % [id, proto])
  peers[id] = true
  peerid = id
  delaytmr = Timer.new()
  delaytmr.wait_time = 2.0
  delaytmr.one_shot = true
  delaytmr.connect("timeout",self,"_on_delaytmr") 
  add_child(delaytmr)
  delaytmr.start()
  print("Started timer")
  
func _on_delaytmr():
  print("Finished delay")
  _setup_ros_listeners(peerid)

func _close_request(id, code, reason):
  print("Client %d disconnecting with code: %d, reason: %s" % [id, code, reason])

func _disconnected(id, was_clean = false):
  peers[id] = false
  print("Client %d disconnected, clean: %s" % [id, str(was_clean)])

func _on_timeout():
  _publish('/vr/alive', true)

func _handle_push_object_3d(args):
  print("TODO handle /3d_object parsing, args: ", args)

func _on_data(id):
  var pkt = _server.get_peer(id).get_packet()
  print("Got data from client %d: %s" % [id, pkt.get_string_from_utf8()])
  
  var result = JSON.parse(pkt.get_string_from_utf8())
  if result.error != OK:
    print("Error parsing: %s" % result.error_string)
    return
  
  match result.result.op:
    "status":
      print("%s: %s" % [result.result.level, result.result.msg])
    "set_level":
      print("set_level: %s" % result.result.level)
    "call_service":
      print("%s: %s" % [result.result.service, result.result.msg])
      if result.result.service == "PushObject3D":
        _handle_push_object_3d(result.result.args)
      else:
        print("Unhandled service call: ", result.result.service)
    _: print("Unhandled ROS bridge op: ", result.result.op)


const PENDANT_MOVE_THRESHOLD_SQUARED = 0.01*0.01
var lastPos = Vector3.ZERO
var debounce = 1.0
func publish_pendant_pos(name: String, pos: Vector3):
  if debounce < 0:
    return
  if (pos-lastPos).length_squared() < PENDANT_MOVE_THRESHOLD_SQUARED:
    return
  # ROS is Z-up
  _publish('/vr/Pendants/Pendant', {
    "x": pos[0],
    "y": pos[2],
    "z": pos[1]
   })
  lastPos = pos
  debounce -= 0.05

func _process(delta):
  debounce = max(debounce + delta, 5.0)
  _server.poll()
