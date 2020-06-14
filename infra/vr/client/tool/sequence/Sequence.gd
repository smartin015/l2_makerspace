extends Spatial

const topic = "sequence"
const topic_type = "l2_msgs/msg/L2Sequence"

remote func set_tf(tf):
  transform = tf

remote func set_ws(ws):
  self.ws = ws
  
func _ready():
  ROSBridge.advertise(topic, topic_type, "run_seq_adv")

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

func _on_SequenceItemUI_run_sequence(seq):
  var items = []
  for s in seq:
    items.append({
      "name": s,
      "params": [],
     })
  ROSBridge.publish(topic, topic_type, {
    "items": items,
  }, "seq")
  print("Published seq to run")

func _on_SequenceItemUI_stop_sequence():
  ROSBridge.publish(topic, topic_type, {
    "items": [],
  }, "seq")
  print("Published stopseq")
