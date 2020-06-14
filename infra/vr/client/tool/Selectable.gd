extends Spatial

enum SelectState {NONE, TRANSLATE, ROTATE, FIXED}
var selected = SelectState.NONE
onready var center = $CenterArea/MeshInstance

remotesync func set_tf(tf):
  get_parent().global_transform = tf

remotesync func set_ws(ws):
  if ws != gamestate.player.ws:
    queue_free()

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

func _on_CenterArea_hit_event(position, click, release):
  if release && visible:
    selected = (selected + 1) % len(SelectState)
    _update_color()
    if selected != SelectState.NONE:
      add_to_group("selection")
    else:
      remove_from_group("selection")
    gamestate.handle_selection_change()

func handle_drag(dest):
  match selected:
    SelectState.TRANSLATE:
      var tf = get_parent().global_transform
      tf.origin = dest
      rpc("set_tf", tf)
    SelectState.ROTATE:
      rpc("set_tf", get_parent().global_transform.looking_at(dest, Vector3.UP))
  selected = SelectState.NONE
  remove_from_group("selection")
  _update_color()
