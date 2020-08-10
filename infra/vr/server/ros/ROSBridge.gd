# Implements a websocket server for connecting to L2 makerspace ros2-web-bridge nodes.
# For rosbridge protocol details, see
# # https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md
extends Node

const WS_PORT = 4243
const RAW_PORT = 4242
const MAX_WS_MSG = 4096

# For isolation, only subscription to topics within
# a certain namespace are allowed for the VR server.
# This should only be visible on the server-side;
# the client should not know this namespace exists.
const NS = "/l2/vr"
func _ns(topic):
  return "%s/%s" % [NS, topic.trim_prefix("/")]
func _strip_ns(topic):
  return topic.trim_prefix(NS+"/")
  
var _server = WebSocketServer.new()
var _raw_server = PacketPeerUDP.new()
var handlers = {} # Server-side handlers or specific topics
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
  for hk in handlers:
    var h = handlers[hk]
    if !h.raw:
      result.push_back({
        "op": "subscribe",
        "topic": h.topic,
        "type": h.type,
        "id": "%s_%s" % [id, h.id],
        "fragment_size": MAX_WS_MSG,
      })
    

  for r in result:
    if r.get("topic"):
      r["topic"] = _ns(r["topic"])
    elif r.get("service"):
      r["service"] = _ns(r["service"])
  return result

func _ready():
  _server.connect("client_connected", self, "_connected")
  _server.connect("client_disconnected", self, "_disconnected")
  _server.connect("client_close_request", self, "_close_request")
  _server.connect("data_received", self, "_on_data")
  if _server.listen(WS_PORT) != OK:
    print("An error occurred listening on port", WS_PORT)
  else:
    print("ROS listen port ", WS_PORT)
  if _raw_server.listen(RAW_PORT, "0.0.0.0") != OK:
    print("An error occurred listening on port", RAW_PORT)
  else:
    print("RAW listen port ", RAW_PORT)

func _prepare_to_send(id: String, msg):
  # msg.id = "%s/%s" % [sender, id]
  msg.id = id
  
  # ALL godot traffic to and from ROS is namespaced so that it's clearly known
  # what topics can be affected by VR users.
  if msg.get('topic'):
    msg.topic = _ns(msg.topic)
  if msg.get('service'):
    msg.service = _ns(msg.service)
  return JSON.print(msg).to_utf8()

func _broadcast(id: String, msg):
  # Broadcast to all ROSPeers
  # Serialization done here instead of in the peers to prevent duplicate work
  var sender = get_tree().get_rpc_sender_id()
  var packet = _prepare_to_send(id, msg)
  for c in get_children():
    if c.should_throttle(sender):
      print("Throttling %s->%s " % [sender, c.name])
      return
    if !c.socket.is_connected_to_host():
      _disconnected(c.name)
      continue
    c.socket.put_packet(packet)

class ROSHandler:
  var topic = ""
  var type = ""
  var id = ""
  var raw = false
  var callbacks = [] # [[node, handler], ...]

# Subscribes to a ROS topic via the server & ros bridge, and registers a 
# handler to call when the topic receives messages.
func ros_connect(topic: String, type: String, node: Node, handler: String, id: String, raw=false):
  var h = handlers.get(topic)
  if !h:
    h = ROSHandler.new()
    handlers[topic] = h
  h.topic = topic
  h.type = type
  h.id = id
  h.callbacks.push_back([node, handler])
  h.raw = raw
  subscribe(topic, type, id, raw)
  return h

func ros_disconnect(h: ROSHandler):
  var th: Array = handlers.get(h.topic, [])
  th.erase(h)

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

remote func subscribe(topic, type, id: String, raw=false):
  var sender = get_tree().get_rpc_sender_id()
  # Ignore repeat subscription
  for sub in listeners.get(topic, []):
    if sub == sender:
      return

  listeners[topic] = listeners.get(topic, []) + [sender]

  if raw:
    # Raw needs no broadcast
    print("RAW listener on %s - now %s" % [topic, listeners[topic]])
    return

  _broadcast(id, {
    "op": "subscribe",
    "topic": topic,
    "type": type,
    "fragment_size": MAX_WS_MSG,
  })
  print("New listener on %s - now %s" % [topic, listeners[topic]])

func unsubscribe(topic, id: String):
  _broadcast(id, {
    "op": "unsubscribe",
    "topic": topic,
  })

remote func call_service(rospeer, service, args, id: String):
  var sender = get_tree().get_rpc_sender_id()
  # TODO how do we direct calls to the right service location?
  # TODO attach a ROSCall object
  if !rospeer.socket.is_connected_to_host():
    _disconnected(rospeer.name)
    return
  var packet = _prepare_to_send(id, {
     "op": "call_service",
     "service": service,
     "args": args,
  })
  calls[id] = sender
  rospeer.socket.put_packet(packet)

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
  var peer = ROSPeer.instance()
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

func _on_raw_data(data):
  if typeof(data) != TYPE_RAW_ARRAY:
    print("ERR invalid raw message of type %s" % typeof(data))
    return
  print(len(data))
  _handle_result(0, {
    "op": "publish",
    "topic": str(data[0]), # First byte is the channel ID
    "msg": data,
  })

func _on_data(id):
  var pkt = _server.get_peer(id).get_packet()
  var result = JSON.parse(pkt.get_string_from_utf8())
  if result.error != OK:
    print("Client %s bad message: %s" % [id, result.error_string])
    return
  _handle_result(id, result.result)

func _handle_result(id, result):
  match result.op:
    "status":
      if result.level != "none":
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
      result.topic = _strip_ns(result.topic)
      var ls = listeners.get(result.topic, [])
      for l in ls:
        var p = gamestate.players.find_node(str(l), false, false)
        if p != null:
          rpc_id(l, "handle_ros_publish", 
            result.get("topic", ""), 
            result.get("msg", ""), 
            result.get("id", ""))
        else:
          # Clear out the listener if the user is no longer present
          print("Clearing unused listener %s" % l)
          ls.erase(l)

      var h = handlers.get(result.topic)
      if h != null:
        for cb in h.callbacks:
          cb[0].call(cb[1], result.get("msg", ""), result.get("id", ""), id)

      if len(ls) == 0 && h == null && !result.topic[0].is_valid_integer():
        # Unsubscribe if nobody's listening and there aren't
        # any handlers. Topics that 
        # start with a number are invalid to ros2-web-bridge
        # and so are assumed to be raw topics (which cannot be
        # unsubscribed)
        print("ROS(%s) unsub %s (unused)" % [id, result.topic])
        unsubscribe(result.topic, "unsub")
    "call_service":
      result.service = _strip_ns(result.service)
      print("ROS(%s) -> call_service %s" % [id, result.service])
      for s in services:
        if s.maybe_handle(result.service, result.id, result.args, id):
          return
      print("ROS(%s) Unhandled call_service: %s" % [id, result.service])
    "service_response":
      result.service = _strip_ns(result.service)
      print("ROS(%s) -> service_response %s" % [id, result.service])
      print("TODO implement service_response")
    _: 
      print("ROS(%s) Unhandled op: ", [id, result.op])

func _process(delta):
  _server.poll()
  if _raw_server.is_listening() && _raw_server.get_available_packet_count() > 0:
    _on_raw_data(_raw_server.get_var())
