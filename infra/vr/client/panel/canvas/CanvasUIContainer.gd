extends VBoxContainer

signal clear
signal save
signal set_shape

func _on_Clear_pressed():
  emit_signal("clear")

func _on_Save_pressed():
  emit_signal("save")

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
