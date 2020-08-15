extends GraphNode

signal edit_pressed
onready var status = $Status
onready var editBtn = $EditButton
onready var paramsList = $Params
var params = {}

func _ready():
  set_slot(0, true, 0, Color(1,1,1,1), true, 0, Color(0,1,0,1), null, null)
  
func _on_SequenceItemNode_close_request():
  rpc("remove")
  
remotesync func remove():
  queue_free()

func _on_SequenceItemNode_dragged(_from, to):
  rpc("set_offset", to)

remotesync func set_offset(offs):
  self.offset = offs

remotesync func set_params(p):
  self.params = p
  
func set_status_label(text):
  status.text = text

func _on_EditButton_pressed():
  emit_signal("edit_pressed", self)

func on_selected():
  # Called by SequenceUI.gd
  editBtn.visible = true

func on_unselected():
  # Called by SequenceUI.gd
 editBtn.visible = false

func get_data():
  var ps = []
  for k in params:
    ps.push_back({"key": k, "value": params[k]})
  return {"name": title, "params": ps}
