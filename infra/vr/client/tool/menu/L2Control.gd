extends Spatial

export(vr.BUTTON) var toggle_button = vr.BUTTON.Y;

var param_edit
var list_select
var ws_control

func _ready():
  param_edit = find_node("L2ParamEdit", true, false)
  list_select = find_node("ListSelect", true, false)
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
      param_edit.fill("Workspace", {
        "name": ws, # TODO remaining fields
      })
      $CenterRaised.visible = true
    "save":
      workspace.snapshot(ws)
    "load":
      list_select.set_list("Load WS", ["test", "test", "test"])
      $CenterRaised2.visible = true
    _:
      print("Unknown action %s for workspace %s" % [action, ws])

func _on_L2SocialControl_social_action(social, action):
  match action:
    "edit":
      param_edit.fill("Client", {
        "name": gamestate.config["alias"],
        "color": gamestate.config["color"],
      })
      $CenterRaised.visible = true
    _:
      print("Unknown action %s for social %s" % [action, social])

func _new_workspace_created(ws):
  ws_control.add_workspace_item(ws)
  param_edit.fill("Workspace", {
    "name": ws
  })
  $CenterRaised.visible = true

func _on_Close_pressed():
  visible = false

func _on_ListSelect_selection():
  $CenterRaised2.visible = false
  visible = false

func _on_ListSelect_delete():
  print("TODO")

func _on_L2ParamEdit_close_edit(title, prev, next):
  print("ParamEdit " + title)
  match title:
    "Workspace":
      print("WS")
      workspace.edit(prev.name, next)
    "Client":
      gamestate.set_social(next)
  $CenterRaised.visible = false
  visible = false 
