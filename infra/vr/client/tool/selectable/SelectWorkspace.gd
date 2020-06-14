extends ReferenceRect

onready var ws_list = $VBoxContainer/ColorRect/ScrollContainer/Workspaces

signal selection

func add_workspace_item(ws):
  var b = Button.new()
  b.text = ws
  b.name = ws
  ws_list.add_child(b)
  b.connect("pressed", self, "_on_workspace_item_pressed",[b])
  
func _ready():
  for ws in workspace.workspaces:
    add_workspace_item(ws)  

func _on_workspace_item_pressed(b):
  emit_signal("selection", b.text)

func _on_Close_pressed():
  emit_signal("selection", null)
