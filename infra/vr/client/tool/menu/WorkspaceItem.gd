extends HBoxContainer

var text setget _on_text_change

signal pressed

func _on_text_change(t: String):
  text = t
  $Button.text = t

func _on_Edit_pressed():
  emit_signal("pressed", "edit")

func _on_Button_pressed():
  emit_signal("pressed", "enter")
