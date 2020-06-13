extends ReferenceRect

export var select_raycast_len = 10.0
onready var tool_select = $VBoxContainer/ColorRect/VBoxContainer/ToolSelect
var selecting = false

func _tool_choice(id):
  print("tool_choice %s" % id)

func _input(ev):
  if ev is InputEventKey and ev.scancode == KEY_K:
    print("kayyyy")

func _on_SelectButton_pressed():
  selecting = !selecting
  
  # update raycast length and allow selection, or revert
  if selecting:
    gamestate.player.set_raycast_len(select_raycast_len)
  else:
    gamestate.player.set_raycast_len(gamestate.player.DEFAULT_RAYCAST_LEN)
  for c in gamestate.tools.get_children():
    var sel = c.find_node("Selectable", true, false)
    if sel != null:
      sel.visible = selecting
