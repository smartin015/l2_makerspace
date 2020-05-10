extends ReferenceRect

export(String) var text setget _set_text, _get_text
onready var label = $MarginContainer/VBoxContainer/RichTextLabel;

func _set_text(s):
  label.text = s
  
func _get_text():
  return label.text

remotesync func clear():
  label.text = ""

func _on_Button_pressed():
  rpc("clear")
