# Implements a websocket server for connecting to L2 makerspace ros2-web-bridge nodes.
# For rosbridge protocol details, see
# # https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md
extends Node

const WS_PORT = 4243
const NS = "/l2/vr"

var _server = WebSocketServer.new()
var listeners = {} # Map of ROS topic to list of godot client IDs
var calls = {} # Map of service call IDs to ROSCalls
var services = []
var topics = []

func connection_msgs(id):
  var result = []
  for s in services:
    result.push_back(s.advertisement(id))
  for t in topics:
    result.push_back(t.advertisement(id))
  return result

func _ready():
  _server.connect("client_connected", self, "_connected")
  _server.connect("client_disconnected", self, "_disconnected")
  _server.connect("client_close_request", self, "_close_request")
  _server.connect("data_received", self, "_on_data")
  if(_server.listen(WS_PORT) != OK):
    print("An error occurred listening on port", WS_PORT)
  else:
    print("ROS listen port ", WS_PORT)

func _broadcast(id: String, msg):
  # Serialization done here instead of in the peers to prevent duplicate work
  var sender = get_tree().get_rpc_sender_id()
  # msg.id = "%s/%s" % [sender, id]
  msg.id = id
  
  # ALL godot traffic to and from ROS is namespaced so that it's clearly known
  # what topics can be affected by VR users.
  if msg.get('topic'):
    msg.topic = "%s/%s" % [NS, msg.topic]
  if msg.get('service'):
    msg.service = "%s/%s" % [NS, msg.service]
  var packet = JSON.print(msg).to_utf8()
  for c in get_children():
    if c.should_throttle(sender):
      print("Throttling %s->%s " % [sender, c.name])
      return
    if !c.socket.is_connected_to_host():
      _disconnected(c.name)
      continue
    c.socket.put_packet(packet)

remote func advertise(topic: String, type: String, id: String):
  _broadcast(id, {
    "op": "advertise",
    "topic": topic,
    "type": type,
  })

remote func publish(topic: String, type: String, msg, id: String):
  _broadcast(id, {
    "op": "publish",
    "topic": topic,
    "type": type,
    "msg": msg,
  })

remote func subscribe(topic, type, id: String):
  listeners[topic] = listeners.get(topic, []) + [get_tree().get_rpc_sender_id()]
  _broadcast(id, {
    "op": "subscribe",
    "topic": topic,
    "type": type,
  })

remote func call_service(service, args, id: String):
  # TODO how do we direct calls to the right service location?
  # TODO attach a ROSCall object
  # _broadcast(id, {
  #   "op": "call_service",
  #   "service": service,
  #   "args": args,
  # })
  pass

func service_response(service, id, values):
  _broadcast(id, {
    "op": "service_response",
    "service": service,
    "values": values,
    "result": true,
   })

func send_ros_peers():
  var peers = []
  for c in get_children():
    peers.push_back(int(c.name))
  rpc('set_ros_peers', peers)

var ROSPeer = load("res://ros/ROSPeer.tscn")
func _connected(id, _proto):
  var peer = ROSPeer.new()
  peer.name = str(id)
  add_child(peer)
  peer.begin_init(id, _server.get_peer(id))
  print("ROS(%d) connected" % id)
  send_ros_peers()
  
func _close_request(id, code, reason):
  print("ROS(%d) disconnecting, code: %d, reason: %s" % [id, code, reason])

func _disconnected(id, was_clean = false):
  var peer = find_node(str(id))
  if !peer:
    return
  remove_child(peer)
  print("ROS(%d) disconnected, clean: %s" % [id, str(was_clean)])
  send_ros_peers()

func _on_data(id):
  var pkt = _server.get_peer(id).get_packet()
  var result = JSON.parse(pkt.get_string_from_utf8())
  if result.error != OK:
    print("Client %s bad message: %s" % [id, result.error_string])
    return
  
  result = result.result
  match result.op:
    "status":
      print("ROS(%s) -> %s %s: %s" % [id, result.id, result.level, result.msg])
      # Forward to player if ID is prefixed with a player
      # var p = gamestate.players.get(result.get(id).split('_')[0])
      # if p != null:
      #    rpc_id(p, "handle_ros_status", 
      #    result.get('level', ''), 
      #     result.get('msg', ''), 
      #     result.get('id', ''))
      #   return
      if result.id != null:
        var name = str(result.id).split('_')[0]
        for c in get_children():
          if c.name == name:
            c.handle_status(result.id, result.level, result.msg)
            print("ROS(%s) %s -> ROSPeer" % [id, result.id])
            break
    "set_level":
      print("ROS(%s) -> set_level: %s" % [id, result.level])
    "publish":
      # forward to subscribed players
      for ls in listeners.get(result.topic, []):
        var p = gamestate.players.get(ls)
        if p != null:
          rpc_id(p, "handle_ros_publish", 
            result.get('topic', ''), 
            result.get('msg', ''), 
            result.get('id'))
    "call_service":
      print("ROS(%s) -> call_service %s" % [id, result.service])
      for s in services:
        if s.maybe_handle(result.service, result.id, result.args, id):
          return
      print("ROS(%s) Unhandled call_service: ", [id, result.service])
      # TODO return with error
    _: print("ROS(%s) Unhandled op: ", [id, result.op])

func _process(delta):
  _server.poll()
