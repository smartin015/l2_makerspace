extends ReferenceRect

var x setget _setx
var y setget _sety
var z setget _setz

onready var xl = $VBoxContainer/ColorRect/ScrollContainer/Params/X
onready var yl = $VBoxContainer/ColorRect/ScrollContainer/Params/Y
onready var zl = $VBoxContainer/ColorRect/ScrollContainer/Params/Z

const fmt = "%6.3f %s"

func _setx(v):
  x = v
  xl.text = fmt % [x, "X"]

func _sety(v):
  y = v
  yl.text = fmt % [y, "Y"]

func _setz(v):
  z = v
  zl.text = fmt % [z, "Z"]
