extends Button

signal create_item
var params

func _on_Button_pressed():
  emit_signal("create_item", text, params)
