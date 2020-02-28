extends Node

# {topic: [[node, handler], ...]}
var handlers = {}

# Subscribes to a ROS topic via the server & ros bridge, and registers a 
# handler to call when the topic receives messages.
func ros_connect(topic: String, type: String, node: Node, handler: String, id: String):
  handlers[topic] = handlers.get(topic, []) + [[node, handler]]
  rpc_id(1, 'subscribe', topic, type, id)

remote func handle_ros_status(level: String, msg: String, id: String):
  print("ROS %s %s: %s" % [level, msg, id])

remote func handle_ros_publish(topic: String, msg, id: String):
  for handler in handlers.get(topic, []):
    handler[0].call(handler, msg, id)

func publish(topic: String, type: String, msg, id: String):
  rpc_id(1, 'publish', topic, type, msg, id)

func advertise(topic: String, type: String, id: String):
  rpc_id(1, 'advertise', topic, type, id)

var healthcheck_timer = null
const HEALTH_CHECK_INTERVAL = 20.0
func _ready():
  healthcheck_timer = Timer.new()
  healthcheck_timer.wait_time = HEALTH_CHECK_INTERVAL
  healthcheck_timer.connect("timeout",self,"_do_ros_healthcheck") 
  add_child(healthcheck_timer)
  healthcheck_timer.start()

func _do_ros_healthcheck():
  if !gamestate.is_initialized:
    return
  var id = str(get_tree().get_network_unique_id())
  ROSBridge.publish('alive', 'std_msgs/String', {"data": id}, "healthcheck")
