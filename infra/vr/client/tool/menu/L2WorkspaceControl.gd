extends ReferenceRect

onready var WorkspaceItem = load("res://tool/menu/WorkspaceItem.tscn")
onready var ws_list = $VBoxContainer/ColorRect/ScrollContainer/Workspaces

signal workspace_action

func add_workspace_item(ws):
  var b = WorkspaceItem.instance()
  b.text = ws
  b.name = ws
  ws_list.add_child(b)
  b.connect("pressed", self, "_on_workspace_item_pressed",[b])
  
func _ready():
  for ws in workspace.workspaces:
    add_workspace_item(ws)  

func _on_workspace_item_pressed(action, b):
  emit_signal("workspace_action", b.text, action)

func _on_NewWorkspace_pressed():
  emit_signal("workspace_action", "", "new")
