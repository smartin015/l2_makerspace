extends Spatial

const objtype = "CONSOLE"
var ws = workspace.DEFAULT
var text = PoolStringArray()
var topic = "console"
var ros = null
var handler = null

func _ready():
  text.push_back("test1")
  text.push_back("test2")
  set_topic(topic)
  
  # Subscribe to get ROS topics
  gamestate.connect("topics_updated", self, "_on_topics_updated")

func _on_topics_updated(topics):
  # Bulk update all clients when new topic details
  rpc("set_topics", topics)

remote func clear():
  text = PoolStringArray()

remote func set_topic(t: String):
  if handler != null:
    ROSBridge.ros_disconnect(handler)
    
  handler = ROSBridge.ros_connect(t, 
    "std_msgs/String", 
    self, "_on_console", 
    "console_sub")
  topic = t
  clear()
  
  # Bulk update all clients
  rpc("setup", topic, text)

func get_state():
  pass

remote func setup_request():
  var sender = get_tree().get_rpc_sender_id()
  rpc_id(sender, "setup", topic, text)
  rpc_id(sender, "set_topics", gamestate.topics)

func _on_console(msg, _id):
  print("Pushed: %s" % [msg])
  text.push_back(msg.data)
  rpc("on_console", msg.data)
  

