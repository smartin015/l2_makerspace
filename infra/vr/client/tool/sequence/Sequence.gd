extends Spatial

var vp

remote func set_tf(tf):
  transform = tf

remote func set_ws(ws):
  self.ws = ws
  
func _ready():
  vp = find_node("Viewport", true, false)
  vp.set_update_mode(Viewport.UPDATE_ONCE)

remote func setup(packed):
  var sui = find_node("SequenceUI", true, false)
  sui.clear()
  var nodes = packed["nodes"]
  for n in nodes:
    var i = sui.create_sequence_item(n[0], n[1])
    i.offset = n[2]
  for conn in packed["connection_list"]:
    sui.connect_node(
      conn["from"], conn["from_port"], conn["to"], conn["to_port"])
  vp.set_update_mode(Viewport.UPDATE_ONCE)
