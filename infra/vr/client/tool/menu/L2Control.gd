extends Spatial

export(vr.BUTTON) var toggle_button = vr.BUTTON.Y;

var ws_edit
var alias_edit
var ws_control

func _ready():
  ws_edit = find_node("L2WorkspaceEdit", true, false)
  alias_edit = find_node("L2AliasEdit", true, false)
  ws_control = find_node("L2WorkspaceControl", true, false)

func _process(_dt):
  if vr.button_just_pressed(toggle_button) || Input.is_action_just_pressed("ui_cycle_mode"):
    visible = false

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
    "save":
      workspace.snapshot(ws)
    _:
      print("Unknown action %s for workspace %s" % [action, ws])

func _on_L2SocialControl_social_action(social, action):
  match action:
    "edit":
      alias_edit.fill({
        "name": gamestate.config["alias"]
      })
      $CenterRaised2.visible = true
    _:
      print("Unknown action %s for social %s" % [action, social])

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
  visible = false 

func _on_Close_pressed():
  visible = false


func _on_L2AliasEdit_close_edit(_prev, next):
  gamestate.set_alias(next["name"])
  $CenterRaised2.visible = false
  # TODO setup so state is updated instead of reloading
  visible = false 
