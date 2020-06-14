extends Node

var handlers = {}
var connected = false
var peers = []
var advertisements = [
  {
    "topic": "alive", 
    "type": "std_msgs/String",
    "id": "healthcheck"
  },
]

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
  rpc_id(1, 'subscribe', topic, type, id, raw)

remote func set_ros_peers(pp):
  print("ROS peers: %s" % [pp])
  if len(pp) > len(peers):
    # Old peers get duplicate adverts, but it should be a no-op
    for a in advertisements:
      advertise(a.topic, a.type, a.id)
    for h in handlers.values():
      rpc_id(1, 'subscribe', h.topic, h.type, h.id, h.raw)
  peers = pp

remote func handle_ros_status(level: String, msg: String, id: String):
  print("ROS %s %s: %s" % [level, msg, id])

remote func handle_ros_publish(topic: String, msg, id: String):
  var h = handlers.get(topic)
  if h == null:
    return
  for cb in h.callbacks:
    cb[0].call(cb[1], msg, id)

func publish(topic: String, type: String, msg, id: String):
  rpc_id(1, 'publish', topic, type, msg, id)

func advertise(topic: String, type: String, id: String):
  rpc_id(1, 'advertise', topic, type, id)

var healthcheck_timer = null
const HEALTH_CHECK_INTERVAL = 5.0
func _ready():
  healthcheck_timer = Timer.new()
  healthcheck_timer.wait_time = HEALTH_CHECK_INTERVAL
  healthcheck_timer.connect("timeout",self,"_do_ros_healthcheck") 
  add_child(healthcheck_timer)
  healthcheck_timer.start()

func _do_ros_healthcheck():
  if len(peers) == 0:
    return
  var id = str(get_tree().get_network_unique_id())
  ROSBridge.publish("alive", "std_msgs/String", {"data": id}, "healthcheck")
