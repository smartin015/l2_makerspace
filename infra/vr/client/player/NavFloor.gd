extends Area

onready var ptr = $Pointer
onready var del = $Delta
var pointing = false
var from_pos = null

func _update_indicators():
  ptr.visible = pointing
  del.visible = pointing and from_pos != null
  if from_pos != null:
    var d = ptr.global_transform.origin - from_pos
    d.y = 0
    del.mesh.size.z = d.length()
    del.transform.origin = d/2 + from_pos
    del.transform.origin.y = 0
    del.transform = del.transform.looking_at(ptr.transform.origin, Vector3.UP)
    
func _ready():
  _update_indicators()

func ui_raycast_hit_event(position, click, release):
  pointing = !release
  if release:
    if from_pos:
      gamestate.move_selection(Vector3(position.x, 0, position.z))
    else:
      gamestate.player.set_origin(Vector3(
        position.x, 
        gamestate.player.transform.origin.y,
        position.z
      ))
  else:
    ptr.transform.origin = Vector3(
      position.x,
      ptr.transform.origin.y,
      position.z
     )
  _update_indicators()
  
