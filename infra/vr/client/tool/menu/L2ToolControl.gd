extends ReferenceRect

export var select_raycast_len = 10.0
onready var tool_select = $VBoxContainer/ColorRect/VBoxContainer/SelectButton
onready var tools = $VBoxContainer/ColorRect/VBoxContainer/Tools
var selecting = false

func _ready():
  for t in gamestate.tools.toolmap:
    # Don't allow user creation of menus
    if t == "MENU":
      continue
    var b = Button.new()
    b.text = t
    b.connect("pressed", self, "_on_tool_pressed", [b])
    tools.add_child(b)

func _on_tool_pressed(b):
  # TODO non-local tool spawning, be less invasive with coords
  var tf = gamestate.player.head.global_transform
  tf = tf.translated(Vector3(0, 0, -0.6))
  var tf2 = Transform()
  tf2.origin = Vector3(tf.origin.x, 0, tf.origin.z)
  # Spawn tool on server (which propagates to all clients where it's visible)
  gamestate.tools.rpc_id(1, "spawn", "newtool", b.text, tf2, gamestate.player.ws)

func _on_SelectButton_pressed():
  selecting = tool_select.is_pressed()
  
  # update raycast length and allow selection, or revert
  if selecting:
    gamestate.player.set_raycast_len(select_raycast_len)
  else:
    gamestate.player.set_raycast_len(gamestate.player.DEFAULT_RAYCAST_LEN)
  for c in gamestate.tools.get_children():
    var sel = c.find_node("Selectable", true, false)
    if sel != null:
      sel.visible = selecting
