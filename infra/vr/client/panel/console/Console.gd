extends Spatial

export var topic = "/console"
var cui

func _ready():
  cui = find_node("ConsoleUI", true, false)
  # Always server-owned
  set_network_master(0)
  
  ROSBridge.ros_connect("console", 
    "std_msgs/String", 
    self, "_on_console", 
    "console_sub")

func _on_console(msg, _id):
  cui.text = cui.text + "\n" + msg.data
