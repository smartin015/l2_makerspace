extends Spatial

export(vr.BUTTON) var toggle_button = vr.BUTTON.Y;

var ws_edit
var ws_control

func _ready():
  ws_edit = find_node("L2WorkspaceEdit", true, false)
  ws_control = find_node("L2WorkspaceControl", true, false)

func _process(_dt):
  if vr.button_just_pressed(toggle_button) || Input.is_action_just_pressed("ui_cycle_mode"):
    queue_free()

func _on_L2WorkspaceControl_workspace_action(ws, action):
  match action:
    "new":
      workspace.request(self, "_new_workspace_created")
    "enter":
      gamestate.set_workspace(ws)
    "edit":
      ws_edit.fill({
        "name": ws
      })
      $CenterRaised.visible = true
    _:
      print("Unknown action %s for workspace %s" % [action, ws])

func _new_workspace_created(ws):
  ws_control.add_workspace_item(ws)
  ws_edit.fill({
    "name": ws
  })
  $CenterRaised.visible = true

func _on_L2WorkspaceEdit_close_edit(prev, next):
  workspace.edit(prev.name, next)
  $CenterRaised.visible = false
  # TODO setup so state is updated instead of reloading
  queue_free() 

func _on_Close_pressed():
  queue_free()
