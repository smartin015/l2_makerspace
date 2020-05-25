extends Spatial

var currLine = {}
var cui
onready var audio = $AudioStreamPlayer2D

func _ready():
  cui = find_node("CanvasUI", true, false)
  cui.connect("gui_input", self, "_on_CanvasUI_gui_input")
  # Always server-owned
  set_network_master(0)

remotesync func clear():
  if cui == null:
    return
  for c in cui.get_children():
    if c is Line2D:
      c.queue_free()

remotesync func handle_input(pressed, position):
  if cui == null:
    return
  var sender = get_tree().get_rpc_sender_id()
  if pressed != null:
    if pressed:
      var nl = Line2D.new()
      cui.add_child(nl)
      nl.add_point(position)
      currLine[sender] = nl
    elif currLine.get(sender) != null:
      currLine[sender] = null
  else:
    var cl = currLine.get(sender)
    if cl != null:
      cl.add_point(position)

func _on_CanvasUI_gui_input(event):
  # Only pass necessary event params to avoid remote
  # code execution from type inference
  # https://github.com/godotengine/godot/issues/28734
  rpc("handle_input", event.get('pressed'), event.get('position'))


func _on_CanvasUIContainer_clear():
  rpc("clear")
  clear()

func _on_CanvasUIContainer_save():
  print("Save")
  audio.play()


func _on_AudioStreamPlayer2D_finished():
  print("Audio finished")
