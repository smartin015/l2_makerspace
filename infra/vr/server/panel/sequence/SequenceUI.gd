extends Control

onready var seqItemNode = load("res://panel/sequence/SequenceItemNode.tscn")
onready var nodes = $MarginContainer/VBoxContainer/Spacer/Nodes

remote func create_sequence_item(n, uid):
  var i = seqItemNode.instance()
  i.name = uid
  i.title = n  
  nodes.add_child(i)
  print("Added node %s" % n)

remote func connect_node(from, from_slot, to, to_slot):
  print("connect_node: %s %s %s %s" % [from, from_slot, to, to_slot])
  nodes.connect_node(from, from_slot, to, to_slot)
