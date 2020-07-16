extends Spatial

enum SelectState {NONE, TRANSLATE, ROTATE, FIXED}
var selected = SelectState.NONE
onready var center = $CenterArea/MeshInstance
onready var tmr = $HoldSelect
onready var SelectableMenu = load("res://tool/selectable/SelectableMenu.tscn")
onready var menu = null

remotesync func set_tf(tf):
  get_parent().global_transform = tf

remotesync func set_ws(ws):
  if ws != gamestate.player.ws:
    get_parent().queue_free()

func _ready():
  center.material_override = SpatialMaterial.new()
  _update_color()
  visible = false # Not visible by default

func _update_color():
  center.material_override.albedo_color = _color()

func _color():
  match selected:
    SelectState.TRANSLATE:
      return Color(0, 0.5, 0)
    SelectState.ROTATE:
      return Color(0.5, 0, 0.5)
    SelectState.FIXED:
      return Color(0, 0.5, 0.5)
    _:
      return Color(0.5, 0.5, 0.5)

var down = false
func _on_CenterArea_hit_event(_position, click, release):
  if !visible:
    return
    
  if click:
    down = true
    tmr.start() # or restart prev
  elif release:
    down = false
    if tmr.is_stopped():
      # Already handled
      return
    tmr.stop()
    selected = (selected + 1) % len(SelectState)
    _update_color()
    if selected != SelectState.NONE:
      add_to_group("selection")
    else:
      remove_from_group("selection")
    gamestate.handle_selection_change()

func _on_HoldSelect_timeout():
  print("Timeout!")
  if menu:
    menu.queue_free()
  var tf = gamestate.player.head.global_transform
  tf = tf.translated(Vector3(0, 0, -0.6))
  var m = SelectableMenu.instance()
  m.transform = tf
  gamestate.tools.add_child(m)
  selected = SelectState.FIXED
  add_to_group("selection")
  _update_color()

func handle_drag(dest):
  match selected:
    SelectState.TRANSLATE:
      var tf = get_parent().global_transform
      tf.origin = dest
      rpc("set_tf", tf)
    SelectState.ROTATE:
      rpc("set_tf", get_parent().global_transform.looking_at(dest, Vector3.UP))
  selected = SelectState.NONE
  print("rm handle_drag")
  remove_from_group("selection")
  _update_color()

func handle_ws(ws):
  rpc("set_ws", ws)

