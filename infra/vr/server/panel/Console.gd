extends Spatial

const objtype = "CONSOLE"
var ws = workspace.DEFAULT
var text = PoolStringArray()
var topic = "console"
var handler = null

func _ready():
  text.push_back("test1")
  text.push_back("test2")
  set_topic(topic)

remote func clear():
  text = PoolStringArray()

remote func set_topic(t):
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

remote func setup_request():
  rpc_id(get_tree().get_rpc_sender_id(), "setup", topic, text)

func _on_console(msg, _id):
  text.push_back(msg.data)
  rpc("on_console", msg.data)
  

