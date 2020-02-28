# Implements a websocket server for connecting to L2 makerspace ros2-web-bridge nodes.
# For rosbridge protocol details, see
# # https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md
extends Node

const WS_PORT = 4243

# TODO: Link this up to message type spec in infra/base/l2_msgs/msg/Object3D.msg
const TYPE_OBJ = 1
const TYPE_SDF = 2
const NS = "/l2/vr"

var ROSPeer = load("res://ROSPeer.gd")
onready var sdf = get_node("/root/World/SDF")

var _server = WebSocketServer.new()
var listeners = {} # Map of ROS topic to list of godot client IDs

func connection_msgs(id):
  return [
    { 
      "op": "advertise_service",
      "service": NS + "/PushObject3D",
      "type": "l2_msgs/PushObject3D",
      "id": "%s_pushobject3d" % id,
    },
  ]

func _ready():
  _server.connect("client_connected", self, "_connected")
  _server.connect("client_disconnected", self, "_disconnected")
  _server.connect("client_close_request", self, "_close_request")
  _server.connect("data_received", self, "_on_data")
  if(_server.listen(WS_PORT) != OK):
    print("An error occurred listening on port", WS_PORT)
  else:
    print("Listening for bridge connections on port ", WS_PORT)

func _broadcast(id: String, msg):
  # Serialization done here instead of in the peers to prevent duplicate work
  var sender = get_tree().get_rpc_sender_id()
  msg.id = "%s/%s" % [sender, id]
  
  # ALL godot traffic to and from ROS is namespaced so that it's clearly known
  # what topics can be affected by VR users.
  if msg.get('topic'):
    msg.topic = "%s/%s" % [NS, msg.topic]
  if msg.get('service'):
    msg.service = "%s/%s" % [NS, msg.service]

  var packet = JSON.print(msg).to_utf8()
  for c in get_children():
    if c.should_throttle(sender):
      return
    c.socket.put_packet(packet)

remote func advertise(topic: String, type: String, msg, id: String):
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

func _service_response(service, id, values):
  _broadcast(id, {
    "op": "service_response",
    "service": service,
    "values": values,
    "result": true,
   })

func _connected(id, proto):
  var peer = ROSPeer.new()
  peer.name = str(id)
  add_child(peer)
  peer.begin_init(id, _server.get_peer(id))
  print("ROS(%d) -> connected" % id)

func _close_request(id, code, reason):
  print("ROS(%d) -> disconnecting with code: %d, reason: %s" % [id, code, reason])

func _disconnected(id, was_clean = false):
  var peer = find_node(str(id))
  if !peer:
    return
  remove_child(peer)
  print("ROS(%d) -> disconnected, clean: %s" % [id, str(was_clean)])

func _handle_push_object_3d(id, args):
  match args.object.type:
    TYPE_OBJ:
      _service_response("PushObject3D", id, {
        "success": false, 
        "message": "OBJ parsing not implemented"
      })
    TYPE_SDF:
      sdf.spawn(args.object.name, args.object.data, Transform.IDENTITY)
      _service_response("PushObject3D", id, {
        "success": true, 
        "message": "SDF model %s created" % args.object.name
      })
    _:
      _service_response("PushObject3D", id, {
        "success": false, 
        "message": "Unknown type %s" % args.object.type
      })

func _on_data(id):
  var pkt = _server.get_peer(id).get_packet()
  var result = JSON.parse(pkt.get_string_from_utf8())
  if result.error != OK:
    print("Client %s bad message: %s" % [id, result.error_string])
    return
  
  result = result.result
  match result.op:
    "status":
      print("ROS(%s) -> %s: %s" % [id, result.level, result.msg])
      # Forward to player if ID is prefixed with a player
      var p = gamestate.players.get(result.get(id).split('_')[0])
      if p != null:
        rpc_id(p, "handle_ros_status", 
          result.get('level', ''), 
          result.get('msg', ''), 
          result.get('id', ''))
        return
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
      if result.service == ('%s/PushObject3D' % NS):
        _handle_push_object_3d(result.result.id, result.args)
      else:
        print("ROS(%s) Unhandled call_service: ", [id, result.service])
    _: print("ROS(%s) Unhandled op: ", [id, result.op])

func _process(delta):
  _server.poll()
