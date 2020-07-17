extends ReferenceRect

onready var WorkspaceItem = load("res://tool/menu/WorkspaceItem.tscn")
onready var ws_list = $VBoxContainer/ColorRect/ScrollContainer/Workspaces
onready var current = $VBoxContainer/HBoxContainer/Current

signal workspace_action

func add_workspace_item(ws):
  var b = WorkspaceItem.instance()
  b.text = ws
  b.name = ws
  ws_list.add_child(b)
  b.connect("pressed", self, "_on_workspace_item_pressed",[b])

func _emit(ws, action):
  emit_signal("workspace_action", ws, action)

func _ready():
  for ws in workspace.workspaces:
    add_workspace_item(ws)
  current.text = "> " + gamestate.player.ws

func _on_workspace_item_pressed(action, b):
  _emit(b.text, action)

func _on_NewWorkspace_pressed():
  _emit("", "new")

func _on_LoadWorkspace_pressed():
  _emit("", "load")

func _on_Save_pressed():
  _emit(gamestate.player.ws, "save")
