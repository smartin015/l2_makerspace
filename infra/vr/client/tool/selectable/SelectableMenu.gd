extends Spatial

func _ready():
  find_node("SelectWorkspace", true, false).set_list("Set Workspace", workspace.workspaces)

func _on_SelectWorkspace_selection(ws):
  if ws != null:
    gamestate.ws_selection(ws)
  queue_free()

func _on_SelectWorkspace_delete():
  gamestate.delete_selection()
  queue_free()
