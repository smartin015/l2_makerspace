extends Control

onready var seqItemNode = load("res://panel/sequence/SequenceItemNode.tscn")
onready var nodes = $MarginContainer/VBoxContainer/Spacer/Nodes

remote func create_sequence_item(n):
  var i = seqItemNode.instance()
  i.title = n  
  nodes.add_child(i)
  print("Added node %s" % n)
