extends VBoxContainer

signal clear
signal save
signal undo
signal redo
signal set_shape

func _on_Clear_pressed():
  emit_signal("clear")

func _on_Save_pressed():
  emit_signal("save")

func _on_Undo_pressed():
  emit_signal("undo")

func _on_Redo_pressed():
  emit_signal("redo")

func _ready():
  var popup = $HBoxContainer/ShapeMenu.get_popup()
  popup.clear()
  for v in range(len(gamestate.SHAPE.values())):
    popup.add_item(str(gamestate.SHAPE.keys()[v]), v)
  #popup.set_position(Vector2(100,100))
  popup.connect("id_pressed", self, "_popupMenuChoice")
  popup.show()

func _popupMenuChoice(id):
  emit_signal("set_shape", id)


func _on_CanvasUI_input_event(_viewport, event, _shape_idx):
  if event is InputEventMouseButton:
    if event.is_pressed():
      print("Object clicky")
    else:
      print("Unclickly")
