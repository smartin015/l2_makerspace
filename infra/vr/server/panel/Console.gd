extends Spatial

const objtype = "CONSOLE"
var workspace = gamestate.DEFAULT_WORKSPACE
var text = PoolStringArray()

func _ready():
  text.push_back("test1")
  text.push_back("test2")
  ROSBridge.ros_connect("console", 
    "std_msgs/String", 
    self, "_on_console", 
    "console_sub")

remote func setup_request():
  rpc_id(get_tree().get_rpc_sender_id(), "setup", text)

func _on_console(msg, _id):
  text.push_back(msg.data)
  rpc("on_console", msg.data)
  

