extends Control

export var text = "Test text" setget _set_text
export var seq_items = [] setget _set_seq_items

# onready var label = $ColorRect/Label
onready var nodes = $MarginContainer/VBoxContainer/Spacer/Nodes
onready var items = $MarginContainer/VBoxContainer/Controls/MarginContainer/Items
onready var seqItemNode = load("res://sequence/SequenceItemNode.tscn")
onready var seqItem = load("res://sequence/SequenceItem.tscn")

func _set_seq_items(si):
  for c in items.get_children():
    c.queue_free()
  for i in si:
    var inst = seqItem.instance()
    inst.text = i
    items.add_child(inst)
  seq_items = items

func _set_text(val):
  text = val
  #if label != null:
  #label.text = val

func _ready():
  _set_text(text)
  _set_seq_items(["test1", "test2"])
  $MarginContainer/VBoxContainer/Spacer/Nodes.get_zoom_hbox().visible = false

func _on_GridContainer_connection_request(from, from_slot, to, to_slot):
  nodes.connect_node(from, from_slot, to, to_slot)


func _on_Left_pressed():
  print("Adding sequence item")
  var i = seqItemNode.instance()
  nodes.add_child(i)
  pass # Replace with function body.


func _on_Right_pressed():
  pass # Replace with function body.


func _on_Left_button_down():
  print("bdown")
  pass # Replace with function body.
