extends Control

onready var seqItemNode = load("res://panel/sequence/SequenceItemNode.tscn")
onready var nodes = $MarginContainer/VBoxContainer/Spacer/Nodes

const topic = "sequence"
const topic_type = "l2_msgs/msg/L2Sequence"

func _ready():
  ROSBridge.advertise(topic, topic_type, "run_seq_adv")

remote func create_sequence_item(n, uid):
  var i = seqItemNode.instance()
  i.name = uid
  i.title = n  
  nodes.add_child(i)
  print("Added node %s" % n)

remote func connect_node(from, from_slot, to, to_slot):
  print("connect_node: %s %s %s %s" % [from, from_slot, to, to_slot])
  nodes.connect_node(from, from_slot, to, to_slot)

remote func run_sequence(cmds):
  var items = []
  for s in cmds:
    items.append({
      "name": s,
      "params": [],
     })
    
  ROSBridge.publish(topic, topic_type, {
    "items": items,
  }, "seq")
  print("Published seq to run: %s" % [cmds])
  
  # TODO remove this fake
  var statmap = {}
  for c in nodes.get_children():
    if c.get("title") == null:
      continue
    if len(cmds) == 0:
      statmap[c.name] = "stopping"
    elif c.title == "test1":
      statmap[c.name] = "test1ing"
    elif c.title == "test2":
      statmap[c.name] = "test2ness"
  rpc("status_update", statmap)
  print("Set fake statuses")
