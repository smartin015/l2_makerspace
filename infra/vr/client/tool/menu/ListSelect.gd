extends ReferenceRect

onready var item_list = $VBoxContainer/ColorRect/ScrollContainer/Items

signal selection
signal delete

func set_list(title, items):
  $VBoxContainer/Label.text = title
  for i in items:
    var b = Button.new()
    b.text = i
    b.name = i
    item_list.add_child(b)
    b.connect("pressed", self, "_on_item_pressed",[b]) 

func _on_item_pressed(b):
  emit_signal("selection", b.text)

func _on_Close_pressed():
  emit_signal("selection", null)

func _on_Delete_pressed():
  emit_signal("delete")
