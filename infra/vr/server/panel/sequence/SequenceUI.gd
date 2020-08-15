extends Control

onready var seqItemNode = load("res://panel/sequence/SequenceItemNode.tscn")
onready var nodes = $MarginContainer/VBoxContainer/Spacer/Nodes

const topic = "Sequence"
const topic_type = "l2_msgs/msg/L2Sequence"

func _ready():
  ROSBridge.ros_connect("SequenceUpdate", 
    "l2_msgs/msg/L2Sequence", 
    self, "_on_sequence_update", 
    "sequence_update_sub")

remote func create_sequence_item(n, uid, params):
  var i = seqItemNode.instance()
  i.name = uid
  i.title = n
  i.params = params
  nodes.add_child(i)
  print("Added node %s" % n)

remote func connect_node(from, from_slot, to, to_slot):
  print("connect_node: %s %s %s %s" % [from, from_slot, to, to_slot])
  nodes.connect_node(from, from_slot, to, to_slot)
  
func pack_state():
  # Pack all node positions, node data/names, and 
  # connections between nodes
  var ns = []
  for n in nodes.get_children():
    if n.get("offset") != null:
      ns.push_back([n.title, n.name, n.offset, n.params])
  return {
    "nodes": ns,
    "connection_list": nodes.get_connection_list(),
  }
  
remote func save():
  var path = "%s_%s.sequence.json" % [name, OS.get_unix_time()]
  ROSBridge.publish("PutFile", "l2_msgs/msg/L2File", {
    "path": path,
    "data": JSON.print(pack_state()),
  }, path)
  
  var sender = get_tree().get_rpc_sender_id()
  # TODO notify write completion
  rpc_id(sender, "on_save", OK, "writing file") 

remote func run_sequence(items):
  ROSBridge.publish(topic, topic_type, {
    "items": items,
  }, "seq")
  print("Published seq to run: %s" % [items])
  
func _on_sequence_update(msg, _id, _peer_id):
  var statmap = {}
  for i in msg.items: 
    statmap[i.uid] = i.status
  rpc("status_update", statmap)
  print("Updated statuses: %s" % statmap)
  
