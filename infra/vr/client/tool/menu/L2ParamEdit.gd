extends ReferenceRect

var orig
onready var attributes = $VBoxContainer/ColorRect/ScrollContainer/Attributes
onready var title = $VBoxContainer/Title
onready var LabelTextEdit = load("res://tool/menu/LabelTextEdit.tscn")
signal close_edit

func _ready():
  pass

func get_form():
  var result = {}
  for c in attributes.get_children():
    var k = c.get_node("Label").text
    result[k] = c.get_node("TextEdit").text
  return result

func set_form(t, f):
  title.text = t
  for c in attributes.get_children():
    c.queue_free()
  for k in f:
    var lte = LabelTextEdit.instance()
    lte.get_node("Label").text = k
    lte.get_node("TextEdit").text = f[k]
    attributes.add_child(lte)

func fill(t, form):
  set_form(t, form)
  orig = get_form()

func _on_Done_pressed():
  emit_signal("close_edit", title.text, orig, get_form())
