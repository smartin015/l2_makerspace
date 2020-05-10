extends Area
func ui_raycast_hit_event(position, click, release):
  if release:
    gamestate.player.transform.origin = Vector3(
        transform.origin.x, 
        gamestate.player.transform.origin.y,
        transform.origin.z
      )
  
