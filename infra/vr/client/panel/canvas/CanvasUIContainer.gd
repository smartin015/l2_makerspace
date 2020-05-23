extends VBoxContainer

signal clear
signal save

func _on_Clear_pressed():
  emit_signal("clear")

func _on_Save_pressed():
  emit_signal("save")
