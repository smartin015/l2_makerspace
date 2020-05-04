extends Spatial

const topic = "sequence"
const topic_type = "l2_msgs/msg/L2Sequence"

func _ready():
  ROSBridge.advertise(topic, topic_type, "run_seq_adv")

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
