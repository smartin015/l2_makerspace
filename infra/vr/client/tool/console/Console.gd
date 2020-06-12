extends Spatial

export var topic = "/console"
var cui

remote func setup(text: PoolStringArray):
  cui.text = ""
  for t in text:
    on_console(t)

func _ready():
  cui = find_node("ConsoleUI", true, false)
  # Always server-owned
  set_network_master(0)
  
remote func on_console(text):
  cui.text = cui.text + "\n" + text
