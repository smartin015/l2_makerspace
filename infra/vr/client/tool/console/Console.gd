extends Spatial

var cui
var vp

func _ready():
  cui = find_node("ConsoleUI", true, false)
  vp = find_node("Viewport", true, false)
  vp.set_update_mode(Viewport.UPDATE_ONCE)
  # Always server-owned
  set_network_master(0)
  
remote func set_topics(topics: PoolStringArray):
  print("set_topics called")
  cui.topics = topics
  
remote func setup(topic: String, text: PoolStringArray):
  cui.set_active_topic(topic)
  cui.text = ""
  for t in text:
    on_console(t)

remotesync func clear():
  cui.text = ""
  vp.set_update_mode(Viewport.UPDATE_ONCE)
  
remote func on_console(text):
  cui.text = cui.text + "\n" + text
  vp.set_update_mode(Viewport.UPDATE_ONCE)

func _on_ConsoleUI_clear():
  rpc("clear")

func _on_ConsoleUI_set_topic(t: String):
  rpc_id(1, "set_topic", t)
