extends Spatial

func _on_SelectWorkspace_selection(ws):
  if ws != null:
    gamestate.ws_selection(ws)
  queue_free()
