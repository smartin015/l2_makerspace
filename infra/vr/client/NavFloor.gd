extends Area

onready var ptr = $Pointer

func ui_raycast_hit_event(position, click, release):
  if release:
    gamestate.player.set_origin(Vector3(
        position.x, 
        gamestate.player.transform.origin.y,
        position.z
      ))
    ptr.visible = false
  else:
    ptr.visible = true
    ptr.transform.origin = Vector3(
      position.x,
      ptr.transform.origin.y,
      position.z
     )
  
