extends Spatial

export var text = "Test text" setget _set_text
onready var ui = $SequenceUIPlane/Viewport/SequenceItemUI

func _set_text(val):
  text = val
  if ui != null:
    ui.text = val

func _ready():
  _set_text(text)
